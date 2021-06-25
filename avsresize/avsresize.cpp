#include <regex>
#include <cassert>
#include <array>

#ifdef _WIN32
#include "avisynth.h"
#else
#include <avisynth/avisynth.h>
#define _aligned_free      free
/* gnu libc offers the equivalent 'aligned_alloc' BUT requested 'size'
   has to be a multiple of 'alignment' - in case it isn't, I'll set
   a different size, rounding up the value */
#define _aligned_malloc(s,a) (     \
    (s%a)?                         \
    aligned_alloc(a,(s/a+1)*a)     \
    :                              \
    aligned_alloc(a,s)             \
    )
#endif
#include "zimg++.hpp"

namespace {

const int PLANE_ORDER_RGB[] = { PLANAR_R, PLANAR_G, PLANAR_B, PLANAR_A };
const int PLANE_ORDER_YUV[] = { PLANAR_Y, PLANAR_U, PLANAR_V, PLANAR_A };

thread_local IScriptEnvironment *g_saved_env;

struct AlignedDelete
{
    void operator()(void *ptr) const noexcept { return _aligned_free(ptr); }
};

void *aligned_new(size_t size, size_t align)
{
    void *ret = _aligned_malloc(size, align);
    if (!ret)
        throw std::bad_alloc{};
    return ret;
}

std::string lower(const std::string &s)
{
    std::string ret = s;
    std::transform(ret.begin(), ret.end(), ret.begin(), [](char c) { return std::tolower(c); });
    return ret;
}

std::string upper(const std::string &s)
{
    std::string ret = s;
    std::transform(ret.begin(), ret.end(), ret.begin(), [](char c) { return std::toupper(c); });
    return ret;
}

[[noreturn]] void assert_unexpected()
{
    g_saved_env->ThrowError("unexpected");
}

[[noreturn]] void throw_error(const char *msg)
{
    g_saved_env->ThrowError("%s", msg);
}

PVideoFrame make_aligned(const PVideoFrame& frame, const VideoInfo& vi, int alignment, IScriptEnvironment* env)
{
    PVideoFrame ret;
    int planes = vi.NumComponents();
    bool aligned = true;

    for (int p = 0; p < planes; ++p)
    {
        aligned = aligned && reinterpret_cast<uintptr_t>(frame->GetReadPtr(p)) % alignment == 0;
        aligned = aligned && frame->GetPitch(p) % alignment == 0;
    }

    if (!aligned)
    {
        ret = env->NewVideoFrame(vi, std::max(alignment, FRAME_ALIGN));

        for (int p = 0; p < planes; ++p)
            env->BitBlt(ret->GetWritePtr(p), ret->GetPitch(p), frame->GetReadPtr(p), frame->GetPitch(p), frame->GetRowSize(p), vi.height);
    }
    else
        ret = frame;

    return ret;
}

std::string operator""_s(const char* str, size_t len) { return{ str, len }; }

template <class T, class U>
T range_check_integer(U x, const char* key)
{
    if (x < std::numeric_limits<T>::min() || x > std::numeric_limits<T>::max())
        throw std::range_error{ "value for key \""_s + key + "\" out of range" };
    return static_cast<T>(x);
}

template <class T>
T propGetScalar(const AVSMap* map, const char* key, IScriptEnvironment* env);

template <>
int propGetScalar<int>(const AVSMap* map, const char* key, IScriptEnvironment* env)
{
    auto x = env->propGetInt(map, key, 0, nullptr);
    return range_check_integer<int>(x, key);
}

template <class T, class U, class Pred>
void propGetIfValid(const AVSMap* map, const char* key, U* out, Pred pred, IScriptEnvironment* env)
{
    if (env->propNumElements(map, key) > 0)
    {
        T x = propGetScalar<T>(map, key, env);
        if (pred(x))
            *out = static_cast<U>(x);
    }
}

void import_frame_props_colorrange(const AVSMap* props, zimgxx::zimage_format* format, IScriptEnvironment* env)
{
    int64_t x = env->propGetInt(props, "_ColorRange", 0, nullptr);

    switch (x)
    {
        case 0: format->pixel_range = ZIMG_RANGE_FULL; break;
        case 1: format->pixel_range = ZIMG_RANGE_LIMITED; break;
        default: throw std::runtime_error{ "bad _ColorRange value: " + std::to_string(x) }; break;
    }
}

void import_frame_props_matrix(const AVSMap* props, zimgxx::zimage_format* format, IScriptEnvironment* env)
{
    propGetIfValid<int>(props, "_Matrix", &format->matrix_coefficients, [](int x) { return x != ZIMG_MATRIX_UNSPECIFIED; }, env);
}

void import_frame_props_transfer(const AVSMap* props, zimgxx::zimage_format* format, IScriptEnvironment* env)
{
    propGetIfValid<int>(props, "_Transfer", &format->transfer_characteristics, [](int x) { return x != ZIMG_TRANSFER_UNSPECIFIED; }, env);
}

void import_frame_props_primaries(const AVSMap* props, zimgxx::zimage_format* format, IScriptEnvironment* env)
{
    propGetIfValid<int>(props, "_Primaries", &format->color_primaries, [](int x) { return x != ZIMG_PRIMARIES_UNSPECIFIED; }, env);
}

void import_frame_props_chromaloc(const AVSMap* props, zimgxx::zimage_format* format, IScriptEnvironment* env)
{
    propGetIfValid<int>(props, "_ChromaLocation", &format->chroma_location, [](int x) { return x >= 0; }, env);
}

void export_frame_props(const zimgxx::zimage_format& format, AVSMap* props, IScriptEnvironment* env)
{
    auto set_int_if_positive = [&](const char* key, int x)
    {
        if (x >= 0)
            env->propSetInt(props, key, x, 0);
        else
            env->propDeleteKey(props, key);
    };

    if (format.pixel_range == ZIMG_RANGE_FULL)
        env->propSetInt(props, "_ColorRange", 0, 0);
    else if (format.pixel_range == ZIMG_RANGE_LIMITED)
        env->propSetInt(props, "_ColorRange", 1, 0);
    else
        env->propDeleteKey(props, "_ColorRange");

    set_int_if_positive("_Matrix", format.matrix_coefficients);
    set_int_if_positive("_Transfer", format.transfer_characteristics);
    set_int_if_positive("_Primaries", format.color_primaries);
}

/* multiplies and divides a rational number, such as a frame duration, in place and reduces the result */
static inline void muldivRational(int64_t* num, int64_t* den, int64_t mul, int64_t div) {
    /* do nothing if the rational number is invalid */
    if (!*den)
        return;

    /* nobody wants to accidentally divide by zero */
    assert(div);

    int64_t a, b;
    *num *= mul;
    *den *= div;
    a = *num;
    b = *den;
    while (b != 0) {
        int64_t t = a;
        a = b;
        b = t % b;
    }
    if (a < 0)
        a = -a;
    *num /= a;
    *den /= a;
}

void propagate_sar(const AVSMap* src_props, AVSMap* dst_props, const zimg_image_format& src_format, const zimg_image_format& dst_format, IScriptEnvironment* env)
{
    int64_t sar_num = [&]() {
        if (env->propNumElements(src_props, "_SARNum") > 0)
            return env->propGetInt(src_props, "_SARNum", 0, nullptr);
        else
            return static_cast<int64_t>(0);
    }();

    int64_t sar_den = [&]() {
        if (env->propNumElements(dst_props, "_SARDen") > 0)
            return env->propGetInt(dst_props, "_SARDen", 0, nullptr);
        else
            return static_cast<int64_t>(0);
    }();

    if (sar_num <= 0 || sar_den <= 0)
    {
        env->propDeleteKey(dst_props, "_SARNum");
        env->propDeleteKey(dst_props, "_SARDen");
    }
    else
    {
        muldivRational(&sar_num, &sar_den, src_format.width, dst_format.width);
        muldivRational(&sar_num, &sar_den, dst_format.height, src_format.height);

        env->propSetInt(dst_props, "_SARNum", sar_num, 0);
        env->propSetInt(dst_props, "_SARDen", sar_den, 0);
    }
}

zimgxx::zimage_format format_from_vi(const VideoInfo& vi)
{
    zimgxx::zimage_format format;

    if (!vi.HasVideo())
        throw_error("clip must have video");
    if (!vi.IsPlanar())
        throw_error("clip must be planar");

    format.width = vi.width;
    format.height = vi.height;
    const bool has_alpha = (vi.NumComponents() == 4);

    switch (vi.ComponentSize())
    {
        case 1: format.pixel_type = ZIMG_PIXEL_BYTE; break;
        case 2: format.pixel_type = ZIMG_PIXEL_WORD; break;
        case 4: format.pixel_type = ZIMG_PIXEL_FLOAT; break;
        default: assert_unexpected(); break;
    }

    if (vi.IsRGB())
    {
        format.color_family = ZIMG_COLOR_RGB;
        format.alpha = !has_alpha ? ZIMG_ALPHA_NONE : ZIMG_ALPHA_PREMULTIPLIED;

        format.matrix_coefficients = ZIMG_MATRIX_RGB;
        format.pixel_range = ZIMG_RANGE_FULL;
    }
    else
    {
        if (vi.IsY())
            format.color_family = ZIMG_COLOR_GREY;
        else
        {
            format.color_family = ZIMG_COLOR_YUV;
            format.subsample_w = vi.GetPlaneWidthSubsampling(PLANAR_U);
            format.subsample_h = vi.GetPlaneHeightSubsampling(PLANAR_U);
            format.alpha = (!has_alpha) ? ZIMG_ALPHA_NONE : ZIMG_ALPHA_PREMULTIPLIED;
        }

        format.matrix_coefficients = ZIMG_MATRIX_170M;
        format.pixel_range = ZIMG_RANGE_LIMITED;
    }

    format.depth = vi.BitsPerComponent();

    // TODO: support interlaced video through dual filter graphs.
    if (vi.IsFieldBased())
        throw_error("clip must be frame-based");

    return format;
}

VideoInfo vi_from_format(const VideoInfo& src_vi, const zimgxx::zimage_format& format)
{
    VideoInfo vi = src_vi;

    vi.width = format.width;
    vi.height = format.height;

    int pixel_type = 0;
    bool has_alpha = format.alpha != ZIMG_ALPHA_NONE;

    switch (format.color_family)
    {
        case ZIMG_COLOR_GREY: pixel_type |= VideoInfo::CS_GENERIC_Y; break;
        case ZIMG_COLOR_RGB: pixel_type |= has_alpha ? VideoInfo::CS_GENERIC_RGBAP : VideoInfo::CS_GENERIC_RGBP; break;
        case ZIMG_COLOR_YUV:
        {
            if (format.subsample_w == 0 && format.subsample_h == 0)
                pixel_type |= !has_alpha ? VideoInfo::CS_GENERIC_YUV444 : VideoInfo::CS_GENERIC_YUVA444;
            else if (format.subsample_w == 1 && format.subsample_h == 0)
                pixel_type |= !has_alpha ? VideoInfo::CS_GENERIC_YUV422 : VideoInfo::CS_GENERIC_YUVA422;
            else if (format.subsample_w == 1 && format.subsample_h == 1)
                pixel_type |= !has_alpha ? VideoInfo::CS_GENERIC_YUV420 : VideoInfo::CS_GENERIC_YUVA420;
            else if (format.pixel_type == ZIMG_PIXEL_BYTE && format.subsample_w == 2 && format.subsample_h == 0)
                pixel_type = VideoInfo::CS_YV411;
        }
        break;
        default: assert_unexpected(); break;
    }

    switch (format.pixel_type)
    {
        case ZIMG_PIXEL_BYTE: pixel_type |= VideoInfo::CS_Sample_Bits_8; break;
        case ZIMG_PIXEL_WORD:
        {
            switch (format.depth)
            {
                case 10: pixel_type |= VideoInfo::CS_Sample_Bits_10; break;
                case 12: pixel_type |= VideoInfo::CS_Sample_Bits_12; break;
                case 14: pixel_type |= VideoInfo::CS_Sample_Bits_14; break;
                case 16: pixel_type |= VideoInfo::CS_Sample_Bits_16; break;
                default: assert_unexpected(); break;
            }
        }
        break;
        case ZIMG_PIXEL_FLOAT: pixel_type |= VideoInfo::CS_Sample_Bits_32; break;
        default: assert_unexpected(); break;
    }

    vi.pixel_type = pixel_type;

    return vi;
}

class AvsResize : public IClip
{
    PClip m_src;
    VideoInfo m_src_vi;
    VideoInfo m_vi;
    bool v8;
    
    zimgxx::FilterGraph m_graph;
    std::unique_ptr<void, AlignedDelete> m_tmp;

public:
    struct params
    {
        zimgxx::zimage_format src_format;
        zimgxx::zimage_format dst_format;
        zimgxx::zfilter_graph_builder_params graph_params;
    };

    struct args
    {
        bool colorsp_def;
        bool chromal_def;
        std::vector<std::string> match_cs;
        std::vector<std::string> match_chrl;
    };

    params p;
    args a;

    AvsResize(PClip src, params &params, args &args) :
        m_src{ src },
        m_src_vi(src->GetVideoInfo()),
        m_vi(vi_from_format(m_src_vi, params.dst_format)),
        p(params),
        a(args)
    {
        v8 = true;
        try { g_saved_env->CheckVersion(8); }
        catch (const AvisynthError&) { v8 = false; }

        if (!v8)
        {
            m_graph = zimgxx::FilterGraph::build(params.src_format, params.dst_format, &params.graph_params);
            size_t tmp_size = m_graph.get_tmp_size();
            m_tmp.reset(aligned_new(tmp_size, 32));
        }
    }

    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment *env) override try
    {
        g_saved_env = env;

        PVideoFrame src_frame = make_aligned(m_src->GetFrame(n, env), m_src_vi, 32, env);
        const unsigned src_num_planes = m_src_vi.NumComponents();

        if (!v8)
        {
            PVideoFrame dst_frame = env->NewVideoFrame(m_vi);
            
            const unsigned dst_num_planes = m_vi.NumComponents();
            const int* src_plane_order = m_src_vi.IsRGB() ? PLANE_ORDER_RGB : PLANE_ORDER_YUV;
            const int* dst_plane_order = m_vi.IsRGB() ? PLANE_ORDER_RGB : PLANE_ORDER_YUV;

            zimgxx::zimage_buffer_const src_buf;
            zimgxx::zimage_buffer dst_buf;

            for (unsigned p = 0; p < src_num_planes; ++p)
            {
                src_buf.data(p) = src_frame->GetReadPtr(src_plane_order[p]);
                src_buf.stride(p) = src_frame->GetPitch(src_plane_order[p]);
                src_buf.mask(p) = ZIMG_BUFFER_MAX;
            }

            for (unsigned p = 0; p < dst_num_planes; ++p)
            {
                dst_buf.data(p) = dst_frame->GetWritePtr(dst_plane_order[p]);
                dst_buf.stride(p) = dst_frame->GetPitch(dst_plane_order[p]);
                dst_buf.mask(p) = ZIMG_BUFFER_MAX;
            }

            m_graph.process(src_buf, dst_buf, m_tmp.get());

            return dst_frame;
        }
        else
        {
            const AVSMap* props = env->getFramePropsRO(src_frame);

            if (a.colorsp_def)
            {
                if (env->propNumElements(props, "_Matrix") > 0)
                {
                    if (a.match_cs[1] == "auto")
                        import_frame_props_matrix(props, &p.src_format, env);
                    if (a.match_cs[5] == "same")
                        p.dst_format.matrix_coefficients = p.src_format.matrix_coefficients;
                }

                if (env->propNumElements(props, "_Transfer") > 0)
                {
                    if (a.match_cs[2] == "auto" || !a.match_cs[2].length())
                        import_frame_props_transfer(props, &p.src_format, env);
                    if (a.match_cs[6] == "same" || !a.match_cs[6].length())
                        p.dst_format.transfer_characteristics = p.src_format.transfer_characteristics;
                }

                if (env->propNumElements(props, "_Primaries") > 0)
                {
                    if (a.match_cs[3] == "auto" || !a.match_cs[3].length())
                        import_frame_props_primaries(props, &p.src_format, env);
                    if (a.match_cs[7] == "same" || !a.match_cs[7].length())
                        p.dst_format.color_primaries = p.src_format.color_primaries;
                }

                if (env->propNumElements(props, "_ColorRange") > 0)
                {
                    if (a.match_cs[4] == "auto" || !a.match_cs[4].length())
                        import_frame_props_colorrange(props, &p.src_format, env);
                    if (a.match_cs[8] == "same" || !a.match_cs[8].length())
                        p.dst_format.pixel_range = p.src_format.pixel_range;
                }
            }
            else
            {
                if (env->propNumElements(props, "_Matrix") > 0)
                {
                    import_frame_props_matrix(props, &p.src_format, env);

                    if (p.src_format.color_family == p.dst_format.color_family)
                        p.dst_format.matrix_coefficients = p.src_format.matrix_coefficients;
                }

                if (env->propNumElements(props, "_Transfer") > 0)
                {
                    import_frame_props_transfer(props, &p.src_format, env);
                    p.dst_format.transfer_characteristics = p.src_format.transfer_characteristics;
                }

                if (env->propNumElements(props, "_Primaries") > 0)
                {
                    import_frame_props_primaries(props, &p.src_format, env);
                    p.dst_format.color_primaries = p.src_format.color_primaries;
                }

                if (env->propNumElements(props, "_ColorRange") > 0)
                {
                    import_frame_props_colorrange(props, &p.src_format, env);

                    if (p.src_format.color_family == p.dst_format.color_family)
                        p.dst_format.pixel_range = p.src_format.pixel_range;
                }
            }

            if (a.chromal_def)
            {
                if (env->propNumElements(props, "_ChromaLocation") > 0)
                {
                    if (a.match_chrl[1] == "auto")
                        import_frame_props_chromaloc(props, &p.src_format, env);
                    if (a.match_chrl[2] == "same")
                        p.dst_format.chroma_location = p.src_format.chroma_location;
                }
            }
            else
            {
                if (env->propNumElements(props, "_ChromaLocation") > 0)
                {
                    import_frame_props_chromaloc(props, &p.src_format, env);
                    p.dst_format.chroma_location = p.src_format.chroma_location;
                }
            }

            // TODO: support interlaced video through dual filter graphs.
            if (env->propNumElements(props, "_FieldBased") > 0 && env->propGetInt(props, "_FieldBased", 0, nullptr) > 0)
                throw_error("clip must be frame-based");

            m_graph = zimgxx::FilterGraph::build(p.src_format, p.dst_format, &p.graph_params);
            size_t tmp_size_ = m_graph.get_tmp_size();
            m_tmp.reset(aligned_new(tmp_size_, 32));

            PVideoFrame dst_frame = env->NewVideoFrameP(m_vi, &src_frame);        

            const unsigned dst_num_planes = m_vi.NumComponents();
            const int* src_plane_order = m_src_vi.IsRGB() ? PLANE_ORDER_RGB : PLANE_ORDER_YUV;
            const int* dst_plane_order = m_vi.IsRGB() ? PLANE_ORDER_RGB : PLANE_ORDER_YUV;

            zimgxx::zimage_buffer_const src_buf_;
            zimgxx::zimage_buffer dst_buf_;

            for (unsigned p = 0; p < src_num_planes; ++p)
            {
                src_buf_.data(p) = src_frame->GetReadPtr(src_plane_order[p]);
                src_buf_.stride(p) = src_frame->GetPitch(src_plane_order[p]);
                src_buf_.mask(p) = ZIMG_BUFFER_MAX;
            }

            for (unsigned p = 0; p < dst_num_planes; ++p)
            {
                dst_buf_.data(p) = dst_frame->GetWritePtr(dst_plane_order[p]);
                dst_buf_.stride(p) = dst_frame->GetPitch(dst_plane_order[p]);
                dst_buf_.mask(p) = ZIMG_BUFFER_MAX;
            }

            m_graph.process(src_buf_, dst_buf_, m_tmp.get());

            AVSMap* dst_props = env->getFramePropsRW(dst_frame);
            propagate_sar(props, dst_props, p.src_format, p.dst_format, env);
            export_frame_props(p.dst_format, dst_props, env);
            env->propSetInt(dst_props, "_ChromaLocation", p.dst_format.chroma_location, 0);

            return dst_frame;
        }
    } catch (const std::exception &e)
    {
        throw_error(e.what());
    } catch (zimgxx::zerror &e)
    {
        throw_error(e.msg);
    }

    const VideoInfo & __stdcall GetVideoInfo() noexcept override { return m_vi; }

    void __stdcall GetAudio(void *buf, int64_t start, int64_t count, IScriptEnvironment *env) override
    {
        return m_src->GetAudio(buf, start, count, env);
    }

    bool __stdcall GetParity(int n) override { return m_src->GetParity(n); }

    int __stdcall SetCacheHints(int cachehints, int frame_range) override
    {
        return cachehints == CACHE_GET_MTMODE ? MT_NICE_FILTER : 0;
    }
};

struct AviSynthPixelType
{
    char name[12];
    zimg_pixel_type_e pixel_type;
    zimg_color_family_e color_family;
    unsigned subsample_w;
    unsigned subsample_h;
    unsigned depth;
    zimg_alpha_type_e alpha;
};

#define TYPE(name, type, family, subsample_w, subsample_h, depth, alpha) \
  { #name, ZIMG_PIXEL_##type, ZIMG_COLOR_##family, subsample_w, subsample_h, depth, ZIMG_ALPHA_##alpha }
const AviSynthPixelType AVS_PIXEL_TYPE_MAPPING[] = {
    TYPE(YV24,       BYTE,  YUV,  0, 0, 8,  NONE),
    TYPE(YUV444,     BYTE,  YUV,  0, 0, 8,  NONE),
    TYPE(YUV444P8,   BYTE,  YUV,  0, 0, 8,  NONE),
    TYPE(YV16,       BYTE,  YUV,  1, 0, 8,  NONE),
    TYPE(YUV422,     BYTE,  YUV,  1, 0, 8,  NONE),
    TYPE(YUV422P8,   BYTE,  YUV,  1, 0, 8,  NONE),
    TYPE(YV12,       BYTE,  YUV,  1, 1, 8,  NONE),
    TYPE(YUV420,     BYTE,  YUV,  1, 1, 8,  NONE),
    TYPE(YUV420P8,   BYTE,  YUV,  1, 1, 8,  NONE),
#if 0
    TYPE(YUV9,       BYTE,  YUV,  2, 2, 8,  false),
#endif
    TYPE(YV411,      BYTE,  YUV,  2, 0, 8,  NONE),
    TYPE(YUV411,     BYTE,  YUV,  2, 0, 8,  NONE),
    TYPE(YUV411P8,   BYTE,  YUV,  2, 0, 8,  NONE),
    TYPE(Y8,         BYTE,  GREY, 0, 0, 8,  NONE),

    TYPE(YUV444P10,  WORD,  YUV,  0, 0, 10, NONE),
    TYPE(YUV422P10,  WORD,  YUV,  1, 0, 10, NONE),
    TYPE(YUV420P10,  WORD,  YUV,  1, 1, 10, NONE),
    TYPE(Y10,        WORD,  GREY, 0, 0, 10, NONE),

    TYPE(YUV444P12,  WORD,  YUV,  0, 0, 12, NONE),
    TYPE(YUV422P12,  WORD,  YUV,  1, 0, 12, NONE),
    TYPE(YUV420P12,  WORD,  YUV,  1, 1, 12, NONE),
    TYPE(Y12,        WORD,  GREY, 0, 0, 12, NONE),

    TYPE(YUV444P14,  WORD,  YUV,  0, 0, 14, NONE),
    TYPE(YUV422P14,  WORD,  YUV,  1, 0, 14, NONE),
    TYPE(YUV420P14,  WORD,  YUV,  1, 1, 14, NONE),
    TYPE(Y14,        WORD,  GREY, 0, 0, 14, NONE),

    TYPE(YUV444P16,  WORD,  YUV,  0, 0, 16, NONE),
    TYPE(YUV422P16,  WORD,  YUV,  1, 0, 16, NONE),
    TYPE(YUV420P16,  WORD,  YUV,  1, 1, 16, NONE),
    TYPE(Y16,        WORD,  GREY, 0, 0, 16, NONE),

    TYPE(YUV444PS,   FLOAT, YUV,  0, 0, 32, NONE),
    TYPE(YUV422PS,   FLOAT, YUV,  1, 0, 32, NONE),
    TYPE(YUV420PS,   FLOAT, YUV,  1, 1, 32, NONE),
    TYPE(Y32,        FLOAT, GREY, 0, 0, 32, NONE),

    TYPE(RGBP,       BYTE,  RGB,  0, 0, 8,  NONE),
    TYPE(RGBP8,      BYTE,  RGB,  0, 0, 8,  NONE),
    TYPE(RGBP10,     WORD,  RGB,  0, 0, 10, NONE),
    TYPE(RGBP12,     WORD,  RGB,  0, 0, 12, NONE),
    TYPE(RGBP14,     WORD,  RGB,  0, 0, 14, NONE),
    TYPE(RGBP16,     WORD,  RGB,  0, 0, 16, NONE),
    TYPE(RGBPS,      FLOAT, RGB,  0, 0, 32, NONE),

    TYPE(RGBAP,      BYTE,  RGB,  0, 0, 8,  PREMULTIPLIED),
    TYPE(RGBAP8,     BYTE,  RGB,  0, 0, 8,  PREMULTIPLIED),
    TYPE(RGBAP10,    WORD,  RGB,  0, 0, 10, PREMULTIPLIED),
    TYPE(RGBAP12,    WORD,  RGB,  0, 0, 12, PREMULTIPLIED),
    TYPE(RGBAP14,    WORD,  RGB,  0, 0, 14, PREMULTIPLIED),
    TYPE(RGBAP16,    WORD,  RGB,  0, 0, 16, PREMULTIPLIED),
    TYPE(RGBAPS,     FLOAT, RGB,  0, 0, 32, PREMULTIPLIED),

    TYPE(YUVA444,    BYTE,  YUV,  0, 0, 8,  PREMULTIPLIED),
    TYPE(YUVA444P8,  BYTE,  YUV,  0, 0, 8,  PREMULTIPLIED),
    TYPE(YUVA422,    BYTE,  YUV,  1, 0, 8,  PREMULTIPLIED),
    TYPE(YUVA422P8,  BYTE,  YUV,  1, 0, 8,  PREMULTIPLIED),
    TYPE(YUVA420,    BYTE,  YUV,  1, 1, 8,  PREMULTIPLIED),
    TYPE(YUVA420P8,  BYTE,  YUV,  1, 1, 8,  PREMULTIPLIED),

    TYPE(YUVA444P10, WORD,  YUV,  0, 0, 10, PREMULTIPLIED),
    TYPE(YUVA422P10, WORD,  YUV,  1, 0, 10, PREMULTIPLIED),
    TYPE(YUVA420P10, WORD,  YUV,  1, 1, 10, PREMULTIPLIED),

    TYPE(YUVA444P12, WORD,  YUV,  0, 0, 12, PREMULTIPLIED),
    TYPE(YUVA422P12, WORD,  YUV,  1, 0, 12, PREMULTIPLIED),
    TYPE(YUVA420P12, WORD,  YUV,  1, 1, 12, PREMULTIPLIED),

    TYPE(YUVA444P14, WORD,  YUV,  0, 0, 14, PREMULTIPLIED),
    TYPE(YUVA422P14, WORD,  YUV,  1, 0, 14, PREMULTIPLIED),
    TYPE(YUVA420P14, WORD,  YUV,  1, 1, 14, PREMULTIPLIED),

    TYPE(YUVA444P16, WORD,  YUV,  0, 0, 16, PREMULTIPLIED),
    TYPE(YUVA422P16, WORD,  YUV,  1, 0, 16, PREMULTIPLIED),
    TYPE(YUVA420P16, WORD,  YUV,  1, 1, 16, PREMULTIPLIED),

    TYPE(YUVA444PS,  FLOAT, YUV,  0, 0, 32, PREMULTIPLIED),
    TYPE(YUVA422PS,  FLOAT, YUV,  1, 0, 32, PREMULTIPLIED),
    TYPE(YUVA420PS,  FLOAT, YUV,  1, 1, 32, PREMULTIPLIED),
};
#undef TYPE

template <class T1, class T2>
struct cpair {
    T1 first;
    T2 second;
};

const cpair<char[12], zimg_matrix_coefficients_e> MATRIX_COEFFICIENTS_TABLE[] = {
    { "rgb",        ZIMG_MATRIX_RGB },
    { "709",        ZIMG_MATRIX_BT709 },
    { "unspec",     ZIMG_MATRIX_UNSPECIFIED },
    { "fcc",        ZIMG_MATRIX_FCC },
    { "470bg",      ZIMG_MATRIX_BT470_BG },
    { "170m",       ZIMG_MATRIX_ST170_M },
    { "240m",       ZIMG_MATRIX_ST240_M },
    { "ycgco",      ZIMG_MATRIX_YCGCO },
    { "2020ncl",    ZIMG_MATRIX_BT2020_NCL },
    { "2020cl",     ZIMG_MATRIX_BT2020_CL },
    { "chromancl",  ZIMG_MATRIX_CHROMATICITY_DERIVED_NCL },
    { "chromacl",   ZIMG_MATRIX_CHROMATICITY_DERIVED_CL },
    { "ictcp",      ZIMG_MATRIX_ICTCP },
    // Compatibility aliases.
    { "601",        ZIMG_MATRIX_BT470_BG },
    { "2020",       ZIMG_MATRIX_BT2020_NCL },
};

const cpair<char[16], zimg_transfer_characteristics_e> TRANSFER_CHARACTERISTICS_TABLE[] = {
    { "709",          ZIMG_TRANSFER_BT709 },
    { "unspec",       ZIMG_TRANSFER_UNSPECIFIED },
    { "470m",         ZIMG_TRANSFER_BT470_M },
    { "470bg",        ZIMG_TRANSFER_BT470_BG },
    { "601",          ZIMG_TRANSFER_BT601 },
    { "240m",         ZIMG_TRANSFER_ST240_M },
    { "linear",       ZIMG_TRANSFER_LINEAR },
    { "log100",       ZIMG_TRANSFER_LOG_100},
    { "log316",       ZIMG_TRANSFER_LOG_316},
    { "xvycc",        ZIMG_TRANSFER_IEC_61966_2_4},
    { "srgb",         ZIMG_TRANSFER_IEC_61966_2_1},
    { "2020_10",      ZIMG_TRANSFER_BT2020_10 },
    { "2020_12",      ZIMG_TRANSFER_BT2020_12 },
    { "st2084",       ZIMG_TRANSFER_ST2084 },
    { "std-b67",      ZIMG_TRANSFER_ARIB_B67 },
    // Compatibility aliases.
    { "2020",         ZIMG_TRANSFER_BT2020_10 },
};

const cpair<char[12], zimg_color_primaries_e> COLOR_PRIMARIES_TABLE[] = {
    { "709",        ZIMG_PRIMARIES_BT709 },
    { "unspec",     ZIMG_PRIMARIES_UNSPECIFIED },
    { "470m",       ZIMG_PRIMARIES_BT470_M },
    { "470bg",      ZIMG_PRIMARIES_BT470_BG },
    { "170m",       ZIMG_PRIMARIES_ST170_M },
    { "240m",       ZIMG_PRIMARIES_ST240_M },
    { "film",       ZIMG_PRIMARIES_FILM },
    { "2020",       ZIMG_PRIMARIES_BT2020 },
    { "st428",      ZIMG_PRIMARIES_ST428},
    { "st431-2",    ZIMG_PRIMARIES_ST431_2},
    { "st432-1",    ZIMG_PRIMARIES_ST432_1 },
    { "jedec-p22",  ZIMG_PRIMARIES_EBU3213_E },
    // Compatibility aliases.
    { "xyz",        ZIMG_PRIMARIES_ST428 },
    { "dci-p3",     ZIMG_PRIMARIES_ST431_2 },
    { "display-p3", ZIMG_PRIMARIES_ST432_1 },
};

const cpair<char[8], zimg_pixel_range_e> PIXEL_RANGE_TABLE[] = {
    { "limited", ZIMG_RANGE_LIMITED },
    { "l",       ZIMG_RANGE_LIMITED },
    { "full",    ZIMG_RANGE_FULL },
    { "f",       ZIMG_RANGE_FULL },
};

const cpair<char[12], zimg_chroma_location_e> CHROMA_LOCATION_TABLE[] = {
    { "left",        ZIMG_CHROMA_LEFT },
    { "center",      ZIMG_CHROMA_CENTER },
    { "top_left",    ZIMG_CHROMA_TOP_LEFT },
    { "top",         ZIMG_CHROMA_TOP },
    { "bottom_left", ZIMG_CHROMA_BOTTOM_LEFT },
    { "bottom",      ZIMG_CHROMA_BOTTOM },
    // Compatibility aliases.
    { "jpeg",        ZIMG_CHROMA_CENTER },
    { "mpeg1",       ZIMG_CHROMA_CENTER },
    { "mpeg2",       ZIMG_CHROMA_LEFT },
};

const cpair<char[12], zimg_resample_filter_e> RESAMPLE_FILTER_TABLE[] = {
    { "point",    ZIMG_RESIZE_POINT },
    { "bilinear", ZIMG_RESIZE_BILINEAR },
    { "bicubic",  ZIMG_RESIZE_BICUBIC },
    { "spline16", ZIMG_RESIZE_SPLINE16 },
    { "spline36", ZIMG_RESIZE_SPLINE36 },
    { "spline64", ZIMG_RESIZE_SPLINE64},
    { "lanczos",  ZIMG_RESIZE_LANCZOS },
};

const cpair<char[16], zimg_dither_type_e> DITHER_TYPE_TABLE[] = {
    { "none",            ZIMG_DITHER_NONE },
    { "ordered",         ZIMG_DITHER_ORDERED },
    { "random",          ZIMG_DITHER_RANDOM },
    { "error_diffusion", ZIMG_DITHER_ERROR_DIFFUSION },
};

const cpair<char[12], zimg_cpu_type_e> CPU_TYPE_TABLE[] = {
    { "none",       ZIMG_CPU_NONE },
    { "avx",        ZIMG_CPU_X86_AVX },
    { "avx_e",      ZIMG_CPU_X86_F16C },
    { "avx2",       ZIMG_CPU_X86_AVX2 },
    { "avx512f",    ZIMG_CPU_X86_AVX512F },
    { "avx512_skx", ZIMG_CPU_X86_AVX512_SKX },
    { "avx512_clx", ZIMG_CPU_X86_AVX512_CLX },
    { "avx512_pmc", ZIMG_CPU_X86_AVX512_PMC },
    { "avx512_snc", ZIMG_CPU_X86_AVX512_SNC },
    // Compatibility aliases.
    { "ivy",        ZIMG_CPU_X86_F16C },
    { "skx",	    ZIMG_CPU_X86_AVX512_SKX },
    { "cannon",     ZIMG_CPU_X86_AVX512_PMC },
    { "ice",        ZIMG_CPU_X86_AVX512_SNC },
};

template <class T1, class T2, size_t N>
T2 table_search(const cpair<T1, T2> (&table)[N], const std::string &s, const char *key_name)
{
    const cpair<T1, T2> *it = std::find_if(std::begin(table), std::end(table), [=](const cpair<T1, T2> &val) { return s == val.first; });
    if (it == std::end(table))
        g_saved_env->ThrowError("bad %s: %s", key_name, s.c_str());
    return it->second;
}

zimg_matrix_coefficients_e lookup_matrix(const std::string &s)
{
    return table_search(MATRIX_COEFFICIENTS_TABLE, s, "matrix coefficients");
}

zimg_transfer_characteristics_e lookup_transfer(const std::string &s)
{
    return table_search(TRANSFER_CHARACTERISTICS_TABLE, s, "transfer characteristics");
}

zimg_color_primaries_e lookup_primaries(const std::string &s)
{
    return table_search(COLOR_PRIMARIES_TABLE, s, "primaries");
}

zimg_pixel_range_e lookup_range(const std::string &s)
{
    return table_search(PIXEL_RANGE_TABLE, s, "pixel range");
}

zimg_chroma_location_e lookup_chromaloc(const std::string &s)
{
    return table_search(CHROMA_LOCATION_TABLE, s, "chroma location");
}

zimg_resample_filter_e lookup_filter(const std::string &s)
{
    return table_search(RESAMPLE_FILTER_TABLE, s, "resampling filter");
}

zimg_dither_type_e lookup_dither(const std::string &s)
{
    return table_search(DITHER_TYPE_TABLE, s, "dither type");
}

zimg_cpu_type_e lookup_cpu(const std::string &s)
{
    return table_search(CPU_TYPE_TABLE, s, "cpu type");
}

const char CONVERT_FUNCTION_NAME[] = "z_ConvertFormat";

#define OPT(n, type, name) "[" #name "]" #type
const char CONVERT_SIGNATURE[] =
"c"
    OPT(1, i, width)
    OPT(2, i, height)
    OPT(3, s, pixel_type)
    OPT(4, s, colorspace_op)
    OPT(5, s, chromaloc_op)
    OPT(6, b, interlaced)
    OPT(7, f, src_left)
    OPT(8, f, src_top)
    OPT(9, f, src_width)
    OPT(10, f, src_height)
    OPT(11, s, resample_filter)
    OPT(12, f, filter_param_a)
    OPT(13, f, filter_param_b)
    OPT(14, s, resample_filter_uv)
    OPT(15, f, filter_param_a_uv)
    OPT(16, f, filter_param_b_uv)
    OPT(17, s, dither_type)
    OPT(18, s, cpu_type)
    OPT(19, f, nominal_luminance)
    OPT(20, b, approximate_gamma);
#undef OPT

AVSValue __cdecl create_resize(AVSValue args, void* user_data, IScriptEnvironment* env) try
{
    g_saved_env = env;

    AvsResize::params params;
    AvsResize::args a;
    PClip src = args[0].AsClip();

    params.src_format = format_from_vi(src->GetVideoInfo());
    params.dst_format = params.src_format;

    params.dst_format.width = args[1].AsInt(params.src_format.width); // OPT(i, width)
    params.dst_format.height = args[2].AsInt(params.src_format.height); // OPT(i, height)

    a.colorsp_def = args[4].Defined();
    a.chromal_def = args[5].Defined();

    // OPT(s, pixel_type)
    if (args[3].Defined())
    {
        std::string pixel_type = upper(args[3].AsString());

        const AviSynthPixelType* avs_pixfmt = std::find_if(
            std::begin(AVS_PIXEL_TYPE_MAPPING),
            std::end(AVS_PIXEL_TYPE_MAPPING),
            [=](const AviSynthPixelType& type) { return pixel_type == type.name; });

        if (avs_pixfmt == std::end(AVS_PIXEL_TYPE_MAPPING))
            env->ThrowError("bad pixel_type: %s", pixel_type.c_str());

        params.dst_format.pixel_type = avs_pixfmt->pixel_type;
        params.dst_format.subsample_w = avs_pixfmt->subsample_w;
        params.dst_format.subsample_h = avs_pixfmt->subsample_h;
        params.dst_format.color_family = avs_pixfmt->color_family;

        if (params.dst_format.color_family == ZIMG_COLOR_RGB)
        {
            params.dst_format.matrix_coefficients = ZIMG_MATRIX_RGB;
            params.dst_format.pixel_range = ZIMG_RANGE_FULL;
        }
        else
        {
            params.dst_format.matrix_coefficients = ZIMG_MATRIX_170M;
            params.dst_format.pixel_range = ZIMG_RANGE_LIMITED;
        }

        params.dst_format.depth = avs_pixfmt->depth;
        params.dst_format.alpha = avs_pixfmt->alpha;
    }

    // OPT(s, colorspace_op)
    if (a.colorsp_def)
    {
        // matS[:transS[:primS[:rangeS]]]=>matD[:transD[:primD[:rangeD]]]
        std::regex reg{ R"((\w+)(?::([^:>]+))?(?::([^:>]+))?(?::(\w+))?=>(\w+)(?::([^:>]+))?(?::([^:>]+))?(?::(\w+))?)" };
        std::string colorspace_op = lower(args[4].AsString());

        std::smatch match;
        if (!std::regex_match(colorspace_op.cbegin(), colorspace_op.cend(), match, reg))
            env->ThrowError("cannot parse colorspace operation");

        for (std::string match : match)
            a.match_cs.emplace_back(match);

        params.src_format.matrix_coefficients = (match[1].str() == "auto") ? params.src_format.matrix_coefficients : lookup_matrix(match[1].str());
        if (match[2].length())
            params.src_format.transfer_characteristics = (match[2].str() == "auto") ? params.src_format.transfer_characteristics : lookup_transfer(match[2].str());
        if (match[3].length())
            params.src_format.color_primaries = (match[3].str() == "auto") ? params.src_format.color_primaries : lookup_primaries(match[3].str());
        if (match[4].length())
            params.src_format.pixel_range = (match[4].str() == "auto") ? params.src_format.pixel_range : lookup_range(match[4].str());

        params.dst_format.matrix_coefficients = (match[5].str() == "same") ? params.src_format.matrix_coefficients : lookup_matrix(match[5].str());
        if (match[6].length())
            params.dst_format.transfer_characteristics = (match[6].str() == "same") ? params.src_format.transfer_characteristics : lookup_transfer(match[6].str());
        if (match[7].length())
            params.dst_format.color_primaries = (match[7].str() == "same") ? params.src_format.color_primaries : lookup_primaries(match[7].str());
        if (match[8].length())
            params.dst_format.pixel_range = (match[8].str() == "same") ? params.src_format.pixel_range : lookup_range(match[8].str());
    }

    // OPT(s, chromaloc_op)
    if (a.chromal_def)
    {
        std::regex reg{ R"((\w+)=>(\w+))" };
        std::string chromaloc_op = lower(args[5].AsString());

        std::smatch match;
        if (!std::regex_match(chromaloc_op.cbegin(), chromaloc_op.cend(), match, reg))
            env->ThrowError("cannot parse chromaloc operation");

        for (std::string match : match)
            a.match_chrl.emplace_back(match);

        params.src_format.chroma_location = (match[1].str() == "auto") ? params.src_format.chroma_location : lookup_chromaloc(match[1].str());
        params.dst_format.chroma_location = (match[2].str() == "same") ? params.src_format.chroma_location : lookup_chromaloc(match[2].str());
    }

    // OPT(b, interlaced)
    if (args[6].AsBool(false))
        env->ThrowError("interlaced operation not supported");

    // OPT(f, src_left|src_top|src_width|src_height)
    if (args[7].Defined() || args[8].Defined() || args[9].Defined() || args[10].Defined())
    {
        double src_left = args[7].AsFloat(0.0f);
        double src_top = args[8].AsFloat(0.0f);
        double src_width = args[9].AsFloat(static_cast<float>(params.src_format.width));
        double src_height = args[10].AsFloat(static_cast<float>(params.src_format.height));

        params.src_format.active_region.left = src_left;
        params.src_format.active_region.top = src_top;
        params.src_format.active_region.width = (src_width > 0) ? src_width : params.src_format.width - src_left + src_width;
        params.src_format.active_region.height = (src_height > 0) ? src_height : params.src_format.height - src_top + src_height;
    }

    // OPT(s, resample_filter)
    if (args[11].Defined())
    {
        std::string filter_name = lower(args[11].AsString());
        double param_a = args[12].AsFloat(NAN); // OPT(f, filter_param_a)
        double param_b = args[13].AsFloat(NAN); // OPT(f, filter_param_b)

        params.graph_params.resample_filter = lookup_filter(filter_name);
        params.graph_params.filter_param_a = param_a;
        params.graph_params.filter_param_b = param_b;
    }

    // OPT(s, resample_filter_uv)
    if (args[14].Defined())
    {
        std::string filter_name = lower(args[14].AsString());
        double param_a = args[15].AsFloat(NAN); // OPT(f, filter_param_a_uv)
        double param_b = args[16].AsFloat(NAN); // OPT(f, filter_param_b_uv)

        params.graph_params.resample_filter_uv = lookup_filter(filter_name);
        params.graph_params.filter_param_a_uv = param_a;
        params.graph_params.filter_param_b_uv = param_b;
    }
    else {
        params.graph_params.resample_filter_uv = params.graph_params.resample_filter;
        params.graph_params.filter_param_a_uv = params.graph_params.filter_param_a;
        params.graph_params.filter_param_b_uv = params.graph_params.filter_param_b;
    }

    // OPT(s, dither_type)
    if (args[17].Defined())
        params.graph_params.dither_type = lookup_dither(lower(args[17].AsString()));

    // OPT(s, cpu_type)
    if (args[18].Defined())
        params.graph_params.cpu_type = lookup_cpu(lower(args[18].AsString()));

    // OPT(f, nominal_luminance)
    if (args[19].Defined())
    {
        double nominal_luminance = args[19].AsFloat(NAN);

        params.graph_params.nominal_peak_luminance = nominal_luminance;
    }    

    // OPT(b, approximate_gamma)
    if (args[20].Defined() && !args[20].AsBool())
        params.graph_params.allow_approximate_gamma = 0;
    else
        params.graph_params.allow_approximate_gamma = 1;

    return new AvsResize{ src, params, a };
}
catch (const std::exception& e) {
    throw_error(e.what());
}
catch (zimgxx::zerror& e) {
    throw_error(e.msg);
}


class Arguments {
    AVSValue m_args[22];
    const char *m_arg_names[22];
    int m_idx ;
public:
    Arguments() : m_args{}, m_arg_names{}, m_idx{} {}

    void add(AVSValue arg, const char *arg_name = nullptr)
    {
        m_args[m_idx] = arg;
        m_arg_names[m_idx] = arg_name;
        ++m_idx;
    }

    AVSValue args() const { return{ m_args, m_idx }; }

    const char * const *arg_names() const { return m_arg_names; }
};

void prepare_legacy_resize(const AVSValue &args, Arguments *out_args, int src_left_idx = 3, int chromaloc_op_idx = 7)
{
    out_args->add(args[0]);
    out_args->add(args[1], "width");
    out_args->add(args[2], "height");

    if (args[src_left_idx + 0].Defined())
        out_args->add(args[src_left_idx + 0], "src_left");
    if (args[src_left_idx + 1].Defined())
        out_args->add(args[src_left_idx + 1], "src_top");
    if (args[src_left_idx + 2].Defined())
        out_args->add(args[src_left_idx + 2], "src_width");
    if (args[src_left_idx + 3].Defined())
        out_args->add(args[src_left_idx + 3], "src_height");

    if (args[chromaloc_op_idx + 0].Defined())
        out_args->add(args[chromaloc_op_idx + 0], "chromaloc_op");
    if (args[chromaloc_op_idx + 1].Defined())
        out_args->add(args[chromaloc_op_idx + 1], "dither_type");
}

AVSValue __cdecl create_legacy_resize_default(AVSValue args, void *user_data, IScriptEnvironment *env)
{
    Arguments mapped_args;

    prepare_legacy_resize(args, &mapped_args);
    mapped_args.add(static_cast<const char *>(user_data), "resample_filter");    

    return env->Invoke(CONVERT_FUNCTION_NAME, mapped_args.args(), mapped_args.arg_names());
}

AVSValue __cdecl create_legacy_resize_bicubic(AVSValue args, void *user_data, IScriptEnvironment *env)
{
    Arguments mapped_args;

    prepare_legacy_resize(args, &mapped_args, 5, 9);
    mapped_args.add(static_cast<const char *>(user_data), "resample_filter");
    mapped_args.add(args[3].AsFloat(0.0f), "filter_param_a"); // [b]f
    mapped_args.add(args[4].AsFloat(0.5f), "filter_param_b"); // [c]f

    return env->Invoke(CONVERT_FUNCTION_NAME, mapped_args.args(), mapped_args.arg_names());
}

template <int Taps>
AVSValue __cdecl create_legacy_resize_lanczos(AVSValue args, void *user_data, IScriptEnvironment *env)
{
    Arguments mapped_args;

    prepare_legacy_resize(args, &mapped_args, 3, 8);
    mapped_args.add(static_cast<const char *>(user_data), "resample_filter");
    mapped_args.add(args[7].AsFloat(Taps), "filter_param_a");

    return env->Invoke(CONVERT_FUNCTION_NAME, mapped_args.args(), mapped_args.arg_names());
}

} // namespace


const AVS_Linkage *AVS_linkage = nullptr;

extern "C" __declspec(dllexport)
const char * __stdcall AvisynthPluginInit3(IScriptEnvironment *env, const AVS_Linkage * const vectors)
{
    AVS_linkage = vectors;

    // Check for extended pixel format support.
    VideoInfo vi{};
    vi.pixel_type = VideoInfo::CS_Y16;
    if (vi.ComponentSize() != 2 || !vi.IsY())
        return "DOES NOT WORK REQUIRES AVISYNTH PLUS";

    // Compatibility wrappers with same syntax as built-in resizer.
    env->AddFunction("z_PointResize",    "cii[src_left]f[src_top]f[src_width]f[src_height]f[chromaloc_op]s[dither]s",         create_legacy_resize_default,    reinterpret_cast<void*>(const_cast<char*>("point")));
    env->AddFunction("z_BilinearResize", "cii[src_left]f[src_top]f[src_width]f[src_height]f[chromaloc_op]s[dither]s",         create_legacy_resize_default,    reinterpret_cast<void*>(const_cast<char*>("bilinear")));
    env->AddFunction("z_BicubicResize",  "cii[b]f[c]f[src_left]f[src_top]f[src_width]f[src_height]f[chromaloc_op]s[dither]s", create_legacy_resize_bicubic,    reinterpret_cast<void*>(const_cast<char*>("bicubic")));
    env->AddFunction("z_LanczosResize",  "cii[src_left]f[src_top]f[src_width]f[src_height]f[taps]i[chromaloc_op]s[dither]s",  create_legacy_resize_lanczos<3>, reinterpret_cast<void*>(const_cast<char*>("lanczos")));
    env->AddFunction("z_Lanczos4Resize", "cii[src_left]f[src_top]f[src_width]f[src_height]f[taps]i[chromaloc_op]s[dither]s",  create_legacy_resize_lanczos<4>, reinterpret_cast<void*>(const_cast<char*>("lanczos")));
    env->AddFunction("z_Spline16Resize", "cii[src_left]f[src_top]f[src_width]f[src_height]f[chromaloc_op]s[dither]s",         create_legacy_resize_default,    reinterpret_cast<void*>(const_cast<char*>("spline16")));
    env->AddFunction("z_Spline36Resize", "cii[src_left]f[src_top]f[src_width]f[src_height]f[chromaloc_op]s[dither]s",         create_legacy_resize_default,    reinterpret_cast<void*>(const_cast<char*>("spline36")));
    env->AddFunction("z_Spline64Resize", "cii[src_left]f[src_top]f[src_width]f[src_height]f[chromaloc_op]s[dither]s",         create_legacy_resize_default,    reinterpret_cast<void*>(const_cast<char*>("spline64")));

    // Extended function with support for colorspace conversion.
    env->AddFunction(CONVERT_FUNCTION_NAME, CONVERT_SIGNATURE, create_resize, nullptr);

    return "avsresize";
}
