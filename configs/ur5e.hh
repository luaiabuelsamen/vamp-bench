#pragma once

#include <vamp/vector.hh>
#include <vamp/vector/math.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots
{
struct UR5e
{
    static constexpr const char *name = "ur5e";
    static constexpr std::size_t dimension = 6;
    static constexpr std::size_t sample_dimension = 6;
    static constexpr std::size_t n_spheres = 27;
    static constexpr float min_radius = 0.0430000014603138;
    static constexpr float max_radius = 0.10000000149011612;
    static constexpr float min_radius_mobile = 0.0430000014603138;
    static constexpr float max_radius_mobile = 0.10000000149011612;
    static constexpr float min_bounding_radius_mobile = 0.0430000014603138;
    static constexpr float max_bounding_radius_mobile = 0.2865024209022522;
    static constexpr std::array<float, 3> base_position = { 0.0, 0.0, 0.16249999403953552 };
    static constexpr std::size_t resolution = 32;
    static constexpr bool euclidean = true;

    static constexpr std::array<std::string_view, dimension> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    static constexpr const char *end_effector = "tool0";

    using Configuration = FloatVector<dimension>;
    using ConfigurationArray = std::array<FloatT, dimension>;
    using Sample = FloatVector<sample_dimension>;
    
    static constexpr std::array<std::size_t, 0> so3_offsets = {  };

    struct alignas(FloatVectorAlignment) ConfigurationBuffer
        : std::array<float, Configuration::num_scalars_rounded>
    {
    };

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, n_spheres> x;
        FloatVector<rake, n_spheres> y;
        FloatVector<rake, n_spheres> z;
        FloatVector<rake, n_spheres> r;
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_m{
        12.566370964050293, 12.566370964050293, 6.2831854820251465, 12.566370964050293, 12.566370964050293, 12.566370964050293
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_a{
        -6.2831854820251465, -6.2831854820251465, -3.1415927410125732, -6.2831854820251465, -6.2831854820251465, -6.2831854820251465
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> d_m{
        0.07957746833562851, 0.07957746833562851, 0.15915493667125702, 0.07957746833562851, 0.07957746833562851, 0.07957746833562851
    };

    static inline void scale_configuration(Configuration& q) noexcept
    {
        q = q * Configuration(s_m) + Configuration(s_a);
    }

    static inline void descale_configuration(Configuration& q) noexcept
    {
        q = (q - Configuration(s_a)) * Configuration(d_m);
    }

    template <std::size_t rake>
    static inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = -6.2831854820251465 + (q[0] * 12.566370964050293);
q[1] = -6.2831854820251465 + (q[1] * 12.566370964050293);
q[2] = -3.1415927410125732 + (q[2] * 6.2831854820251465);
q[3] = -6.2831854820251465 + (q[3] * 12.566370964050293);
q[4] = -6.2831854820251465 + (q[4] * 12.566370964050293);
q[5] = -6.2831854820251465 + (q[5] * 12.566370964050293);

    }

    template <std::size_t rake>
    static inline void descale_configuration_block(ConfigurationBlock<rake> & q) noexcept
    {
        q[0] = 0.07957746833562851 * (q[0] - -6.2831854820251465);
q[1] = 0.07957746833562851 * (q[1] - -6.2831854820251465);
q[2] = 0.15915493667125702 * (q[2] - -3.1415927410125732);
q[3] = 0.07957746833562851 * (q[3] - -6.2831854820251465);
q[4] = 0.07957746833562851 * (q[4] - -6.2831854820251465);
q[5] = 0.07957746833562851 * (q[5] - -6.2831854820251465);

    }

    inline static auto space_measure() noexcept -> float
    {
        return 1968925.5;
    }

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> lower_bound{
        -6.2831854820251465, -6.2831854820251465, -3.1415927410125732, -6.2831854820251465, -6.2831854820251465, -6.2831854820251465
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> upper_bound{
        6.2831854820251465, 6.2831854820251465, 3.1415927410125732, 6.2831854820251465, 6.2831854820251465, 6.2831854820251465
    };

    static inline auto in_bounds(const Configuration &x) -> bool
    {
        return (x <= Configuration(upper_bound)).all() and (x >= Configuration(lower_bound)).all();
    }

    static inline auto sample(const Sample &x_in) -> Configuration
    {
        
        // Euclidean fast path: same affine map as scale_configuration,
        // operating on the packed FloatVector in one SIMD step. Sample and
        // Configuration alias to the same FloatVector type when Euclidean.
        Configuration q = x_in;
        scale_configuration(q);
        return q;
        
    }

    static inline auto distance(const Configuration &a_in, const Configuration &b_in) -> float
    {
        
        return a_in.distance(b_in);
        
    }

    static inline auto interpolate(const Configuration &a_in, const Configuration &b_in, float t) -> Configuration
    {
        
        return a_in.interpolate(b_in, t);
        
    }

    template <std::size_t rake>
    static inline void interpolate_block(
        const Configuration &a,
        const Configuration &b,
        const FloatVector<rake> &t,
        ConfigurationBlock<rake> &out) noexcept
    {
        // V is referenced by the SIMD-mask form emitted by LanguageCVampBlock
        // for non-Euclidean robots; constants and operands get wrapped as
        // V(x).blend(V(y), (V(...) CMP V(...))) so .blend() always has a
        // vector receiver.
        using V = FloatVector<rake, 1>;
        
           out[0] = a.broadcast(0) + t * (b.broadcast(0) - a.broadcast(0));
   out[1] = a.broadcast(1) + t * (b.broadcast(1) - a.broadcast(1));
   out[2] = a.broadcast(2) + t * (b.broadcast(2) - a.broadcast(2));
   out[3] = a.broadcast(3) + t * (b.broadcast(3) - a.broadcast(3));
   out[4] = a.broadcast(4) + t * (b.broadcast(4) - a.broadcast(4));
   out[5] = a.broadcast(5) + t * (b.broadcast(5) - a.broadcast(5));

    }

    template <std::size_t rake>
    static inline void sphere_fk(const ConfigurationBlock<rake> &x, Spheres<rake> &out) noexcept
    {
        std::array<FloatVector<rake, 1>, 21> v;
        std::array<FloatVector<rake, 1>, 108> y;

           v[0] = cos(x[0]);
   v[1] = sin(x[0]);
   v[2] = -1. * v[0] + -1.22464679914735e-16 * v[1];
   v[3] = cos(x[1]);
   v[1] = 1.22464679914735e-16 * v[0] + -1. * v[1];
   v[0] = 0. - v[1];
   v[4] = sin(x[1]);
   v[5] = -2.05103489747671e-10 * v[4];
   v[6] = v[2] * v[3] + v[0] * v[5];
   v[7] = -1. * v[0];
   y[4] = -0.416 * v[6] + 0.143 * v[7];
   v[8] = v[1] * v[3] + v[2] * v[5];
   v[9] = -1. * v[2];
   y[5] = -0.416 * v[8] + 0.143 * v[9];
   y[6] = 0.1625 + -2.9329799033917e-11 + -0.416 * v[4];
   y[8] = -0.015 * v[6] + 0.134 * v[7];
   y[9] = -0.015 * v[8] + 0.134 * v[9];
   y[10] = 0.1625 + -2.74838676261879e-11 + -0.015 * v[4];
   y[12] = -0.14 * v[6] + 0.138 * v[7];
   y[13] = -0.14 * v[8] + 0.138 * v[9];
   y[14] = 0.1625 + -2.83042815851786e-11 + -0.14 * v[4];
   v[5] = -1. * v[4] - -2.05103489747671e-10 * v[5];
   v[10] = -2.05103489747671e-10 * v[3];
   v[0] = v[2] * v[5] + v[0] * v[10];
   y[16] = -0.285 * v[6] + -0.001 * v[0] + 0.139 * v[7];
   v[10] = v[1] * v[5] + v[2] * v[10];
   y[17] = -0.285 * v[8] + -0.001 * v[10] + 0.139 * v[9];
   v[3] = 0. - -1. * v[3];
   y[18] = 0.1625 + -0.285 * v[4] + -2.85093850749263e-11 + -0.001 * v[3];
   y[20] = -0.376 * v[6] + 0.001 * v[0] + 0.138 * v[7];
   y[21] = -0.376 * v[8] + 0.001 * v[10] + 0.138 * v[9];
   y[22] = 0.1625 + -0.376 * v[4] + -2.83042815851786e-11 + 0.001 * v[3];
   y[24] = -0.222 * v[6] + 0.001 * v[0] + 0.139 * v[7];
   y[25] = -0.222 * v[8] + 0.001 * v[10] + 0.139 * v[9];
   y[26] = 0.1625 + -0.222 * v[4] + -2.85093850749263e-11 + 0.001 * v[3];
   y[28] = -0.055 * v[6] + 0.008 * v[0] + 0.14 * v[7];
   y[29] = -0.055 * v[8] + 0.008 * v[10] + 0.14 * v[9];
   y[30] = 0.1625 + -0.055 * v[4] + -2.8714488564674e-11 + 0.008 * v[3];
   y[32] = -0.001 * v[6] + -0.002 * v[0] + 0.143 * v[7];
   y[33] = -0.001 * v[8] + -0.002 * v[10] + 0.143 * v[9];
   y[34] = 0.1625 + -0.001 * v[4] + -2.9329799033917e-11 + -0.002 * v[3];
   v[5] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = v[6] * v[5] + v[0] * v[1];
   v[11] = 0. - v[1];
   v[0] = v[6] * v[11] + v[0] * v[5];
   v[6] = -0.425 * v[6];
   y[36] = -0.01 * v[2] + 0.002 * v[0] + 0.031 * v[7] + v[6];
   v[12] = v[8] * v[5] + v[10] * v[1];
   v[10] = v[8] * v[11] + v[10] * v[5];
   v[8] = -0.425 * v[8];
   y[37] = -0.01 * v[12] + 0.002 * v[10] + 0.031 * v[9] + v[8];
   v[1] = v[4] * v[5] + v[3] * v[1];
   v[11] = v[4] * v[11] + v[3] * v[5];
   v[4] = 0.1625 + -0.425 * v[4];
   y[38] = -0.01 * v[1] + -6.3582081821778e-12 + 0.002 * v[11] + v[4];
   y[40] = -0.387 * v[2] + 0.014 * v[7] + v[6];
   y[41] = -0.387 * v[12] + 0.014 * v[9] + v[8];
   y[42] = -2.8714488564674e-12 + -0.387 * v[1] + v[4];
   y[44] = -0.121 * v[2] + 0.006 * v[7] + v[6];
   y[45] = -0.121 * v[12] + 0.006 * v[9] + v[8];
   y[46] = -1.23062093848603e-12 + -0.121 * v[1] + v[4];
   y[48] = -0.206 * v[2] + 0.001 * v[0] + 0.007 * v[7] + v[6];
   y[49] = -0.206 * v[12] + 0.001 * v[10] + 0.007 * v[9] + v[8];
   y[50] = -0.206 * v[1] + -1.4357244282337e-12 + 0.001 * v[11] + v[4];
   y[52] = -0.312 * v[2] + -0.001 * v[0] + 0.006 * v[7] + v[6];
   y[53] = -0.312 * v[12] + -0.001 * v[10] + 0.006 * v[9] + v[8];
   y[54] = -0.312 * v[1] + -1.23062093848603e-12 + -0.001 * v[11] + v[4];
   y[56] = -0.057 * v[2] + 0.003 * v[0] + 0.008 * v[7] + v[6];
   y[57] = -0.057 * v[12] + 0.003 * v[10] + 0.008 * v[9] + v[8];
   y[58] = -0.057 * v[1] + -1.64082791798137e-12 + 0.003 * v[11] + v[4];
   y[60] = -0.266 * v[2] + 0.006 * v[7] + v[6];
   y[61] = -0.266 * v[12] + 0.006 * v[9] + v[8];
   y[62] = -1.23062093848603e-12 + -0.266 * v[1] + v[4];
   y[64] = -0.397 * v[2] + -0.001 * v[0] + -0.018 * v[7] + v[6];
   y[65] = -0.397 * v[12] + -0.001 * v[10] + -0.018 * v[9] + v[8];
   y[66] = -0.397 * v[1] + 3.69186281545808e-12 + -0.001 * v[11] + v[4];
   y[68] = -0.164 * v[2] + 0.007 * v[7] + v[6];
   y[69] = -0.164 * v[12] + 0.007 * v[9] + v[8];
   y[70] = -1.4357244282337e-12 + -0.164 * v[1] + v[4];
   v[6] = v[6] + -0.3922 * v[2] + 0.1333 * v[7];
   y[72] = -0.009 * v[7] + v[6];
   v[8] = v[8] + -0.3922 * v[12] + 0.1333 * v[9];
   y[73] = -0.009 * v[9] + v[8];
   v[4] = v[4] + -2.73402951833646e-11 + -0.3922 * v[1];
   y[74] = 1.84593140772904e-12 + v[4];
   y[76] = -0.052 * v[7] + v[6];
   y[77] = -0.052 * v[9] + v[8];
   y[78] = 1.06653814668789e-11 + v[4];
   v[5] = cos(x[3]);
   v[3] = sin(x[3]);
   v[13] = v[2] * v[5] + v[0] * v[3];
   v[14] = 0. - v[3];
   v[0] = v[2] * v[14] + v[0] * v[5];
   y[80] = -0.002 * v[13] + 0.027 * v[0] + -0.001 * v[7] + v[6];
   v[2] = v[12] * v[5] + v[10] * v[3];
   v[10] = v[12] * v[14] + v[10] * v[5];
   y[81] = -0.002 * v[2] + 0.027 * v[10] + -0.001 * v[9] + v[8];
   v[3] = v[1] * v[5] + v[11] * v[3];
   v[14] = v[1] * v[14] + v[11] * v[5];
   y[82] = -0.002 * v[3] + 2.05103489747671e-13 + 0.027 * v[14] + v[4];
   y[84] = 0.001 * v[13] + -0.01 * v[0] + v[6];
   y[85] = 0.001 * v[2] + -0.01 * v[10] + v[8];
   y[86] = 0.001 * v[3] + -0.01 * v[14] + v[4];
   v[5] = sin(x[4]);
   v[11] = -2.05103489747671e-10 * v[5];
   v[1] = -1. * v[5] - -2.05103489747671e-10 * v[11];
   v[12] = cos(x[4]);
   v[15] = -2.05103489747671e-10 * v[12];
   v[16] = 0. - -1. * v[12];
   v[17] = v[13] * v[1] + v[0] * v[15] + v[7] * v[16];
   v[18] = -1. * v[0] + -2.05103489747671e-10 * v[7];
   v[6] = v[6] + -0.0997 * v[0] + -2.04488118229785e-11 * v[7];
   y[88] = -0.01 * v[17] + -0.001 * v[18] + v[6];
   v[19] = v[2] * v[1] + v[10] * v[15] + v[9] * v[16];
   v[20] = -1. * v[10] + -2.05103489747671e-10 * v[9];
   v[8] = v[8] + -0.0997 * v[10] + -2.04488118229785e-11 * v[9];
   y[89] = -0.01 * v[19] + -0.001 * v[20] + v[8];
   v[16] = v[3] * v[1] + v[14] * v[15] + -2.05103489747671e-10 * v[16];
   v[15] = 4.2067441506673e-20 + -1. * v[14];
   v[4] = v[4] + 4.19412266608633e-21 + -0.0997 * v[14];
   y[90] = -0.01 * v[16] + -0.001 * v[15] + v[4];
   y[92] = 0.008 * v[17] + -0.001 * v[18] + v[6];
   y[93] = 0.008 * v[19] + -0.001 * v[20] + v[8];
   y[94] = 0.008 * v[16] + -0.001 * v[15] + v[4];
   v[0] = v[13] * v[12] + v[0] * v[11] + v[7] * v[5];
   y[96] = 0.001 * v[0] + -0.001 * v[17] + -0.036 * v[18] + v[6];
   v[10] = v[2] * v[12] + v[10] * v[11] + v[9] * v[5];
   y[97] = 0.001 * v[10] + -0.001 * v[19] + -0.036 * v[20] + v[8];
   v[12] = v[3] * v[12] + v[14] * v[11] + -2.05103489747671e-10 * v[5];
   y[98] = 0.001 * v[12] + -0.001 * v[16] + -0.036 * v[15] + v[4];
   y[100] = 0.001 * v[0] + -0.03 * v[17] + v[6];
   y[101] = 0.001 * v[10] + -0.03 * v[19] + v[8];
   y[102] = 0.001 * v[12] + -0.03 * v[16] + v[4];
   v[11] = cos(x[5]);
   v[5] = sin(x[5]);
   v[14] = v[11] + -1.22464679939853e-16 * v[5];
   v[3] = -1.22464679914735e-16 * v[11] + -2.05103489747671e-10 * v[5];
   v[5] = -1.22464679914735e-16 * v[11] + -1. * v[5];
   v[11] = v[5] - -2.05103489747671e-10 * v[3];
   v[2] = -2.05103489747671e-10 * v[14] - 1.22464679889617e-16 * v[5];
   v[9] = 1.22464679889617e-16 * v[3] - v[14];
   y[104] = 0.001 * (v[0] * v[14] + v[17] * v[3] + v[18] * v[5]) + 0.001 * (v[0] * v[11] + v[17] * v[2] + v[18] * v[9]) + -0.029 * (1.22464679889617e-16 * v[0] + v[17] + -2.05103489747671e-10 * v[18]) + v[6] + 0.0996 * v[17] + -2.0428301480127e-11 * v[18];
   y[105] = 0.001 * (v[10] * v[14] + v[19] * v[3] + v[20] * v[5]) + 0.001 * (v[10] * v[11] + v[19] * v[2] + v[20] * v[9]) + -0.029 * (1.22464679889617e-16 * v[10] + v[19] + -2.05103489747671e-10 * v[20]) + v[8] + 0.0996 * v[19] + -2.0428301480127e-11 * v[20];
   y[106] = 0.001 * (v[12] * v[14] + v[16] * v[3] + v[15] * v[5]) + 0.001 * (v[12] * v[11] + v[16] * v[2] + v[15] * v[9]) + -0.029 * (1.22464679889617e-16 * v[12] + v[16] + -2.05103489747671e-10 * v[15]) + v[4] + 0.0996 * v[16] + -2.0428301480127e-11 * v[15];
   // dependent variables without operations
   y[0] = 0.;
   y[1] = 0.;
   y[2] = 0.1625;
   y[3] = 0.100000001490116;
   y[7] = 0.0780000016093254;
   y[11] = 0.0769999995827675;
   y[15] = 0.061999998986721;
   y[19] = 0.0610000006854534;
   y[23] = 0.0769999995827675;
   y[27] = 0.0610000006854534;
   y[31] = 0.0700000002980232;
   y[35] = 0.0799999982118607;
   y[39] = 0.0719999969005585;
   y[43] = 0.0570000000298023;
   y[47] = 0.0570000000298023;
   y[51] = 0.0570000000298023;
   y[55] = 0.0560000017285347;
   y[59] = 0.0649999976158142;
   y[63] = 0.0570000000298023;
   y[67] = 0.0520000010728836;
   y[71] = 0.0570000000298023;
   y[75] = 0.0469999983906746;
   y[79] = 0.0469999983906746;
   y[83] = 0.0450000017881393;
   y[87] = 0.046000000089407;
   y[91] = 0.0469999983906746;
   y[95] = 0.0469999983906746;
   y[99] = 0.0469999983906746;
   y[103] = 0.0469999983906746;
   y[107] = 0.0430000014603138;


        for (auto i = 0U; i < 27; ++i)
        {
            out.x[i] = y[i * 4 + 0];
            out.y[i] = y[i * 4 + 1];
            out.z[i] = y[i * 4 + 2];
            out.r[i] = y[i * 4 + 3];
        }
    }

    using Debug = std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    template <std::size_t rake>
        static inline auto fkcc_debug(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept -> Debug
    {
        std::array<FloatVector<rake, 1>, 52> v;
        std::array<FloatVector<rake, 1>, 132> y;

           v[0] = cos(x[0]);
   v[1] = sin(x[0]);
   v[2] = -1. * v[0] + -1.22464679914735e-16 * v[1];
   v[3] = cos(x[1]);
   v[1] = 1.22464679914735e-16 * v[0] + -1. * v[1];
   v[0] = 0. - v[1];
   v[4] = sin(x[1]);
   v[5] = -2.05103489747671e-10 * v[4];
   v[6] = v[2] * v[3] + v[0] * v[5];
   v[7] = -1. * v[0];
   y[4] = -0.416 * v[6] + 0.143 * v[7];
   v[8] = v[1] * v[3] + v[2] * v[5];
   v[9] = -1. * v[2];
   y[5] = -0.416 * v[8] + 0.143 * v[9];
   y[6] = 0.1625 + -2.9329799033917e-11 + -0.416 * v[4];
   y[8] = -0.015 * v[6] + 0.134 * v[7];
   y[9] = -0.015 * v[8] + 0.134 * v[9];
   y[10] = 0.1625 + -2.74838676261879e-11 + -0.015 * v[4];
   y[12] = -0.14 * v[6] + 0.138 * v[7];
   y[13] = -0.14 * v[8] + 0.138 * v[9];
   y[14] = 0.1625 + -2.83042815851786e-11 + -0.14 * v[4];
   v[5] = -1. * v[4] - -2.05103489747671e-10 * v[5];
   v[10] = -2.05103489747671e-10 * v[3];
   v[0] = v[2] * v[5] + v[0] * v[10];
   y[16] = -0.285 * v[6] + -0.001 * v[0] + 0.139 * v[7];
   v[10] = v[1] * v[5] + v[2] * v[10];
   y[17] = -0.285 * v[8] + -0.001 * v[10] + 0.139 * v[9];
   v[3] = 0. - -1. * v[3];
   y[18] = 0.1625 + -0.285 * v[4] + -2.85093850749263e-11 + -0.001 * v[3];
   y[20] = -0.376 * v[6] + 0.001 * v[0] + 0.138 * v[7];
   y[21] = -0.376 * v[8] + 0.001 * v[10] + 0.138 * v[9];
   y[22] = 0.1625 + -0.376 * v[4] + -2.83042815851786e-11 + 0.001 * v[3];
   y[24] = -0.222 * v[6] + 0.001 * v[0] + 0.139 * v[7];
   y[25] = -0.222 * v[8] + 0.001 * v[10] + 0.139 * v[9];
   y[26] = 0.1625 + -0.222 * v[4] + -2.85093850749263e-11 + 0.001 * v[3];
   y[28] = -0.055 * v[6] + 0.008 * v[0] + 0.14 * v[7];
   y[29] = -0.055 * v[8] + 0.008 * v[10] + 0.14 * v[9];
   y[30] = 0.1625 + -0.055 * v[4] + -2.8714488564674e-11 + 0.008 * v[3];
   y[32] = -0.001 * v[6] + -0.002 * v[0] + 0.143 * v[7];
   y[33] = -0.001 * v[8] + -0.002 * v[10] + 0.143 * v[9];
   y[34] = 0.1625 + -0.001 * v[4] + -2.9329799033917e-11 + -0.002 * v[3];
   v[5] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = v[6] * v[5] + v[0] * v[1];
   v[11] = 0. - v[1];
   v[12] = v[6] * v[11] + v[0] * v[5];
   v[13] = -0.425 * v[6];
   y[36] = -0.01 * v[2] + 0.002 * v[12] + 0.031 * v[7] + v[13];
   v[14] = v[8] * v[5] + v[10] * v[1];
   v[15] = v[8] * v[11] + v[10] * v[5];
   v[16] = -0.425 * v[8];
   y[37] = -0.01 * v[14] + 0.002 * v[15] + 0.031 * v[9] + v[16];
   v[1] = v[4] * v[5] + v[3] * v[1];
   v[11] = v[4] * v[11] + v[3] * v[5];
   v[5] = 0.1625 + -0.425 * v[4];
   y[38] = -0.01 * v[1] + -6.3582081821778e-12 + 0.002 * v[11] + v[5];
   y[40] = -0.387 * v[2] + 0.014 * v[7] + v[13];
   y[41] = -0.387 * v[14] + 0.014 * v[9] + v[16];
   y[42] = -2.8714488564674e-12 + -0.387 * v[1] + v[5];
   y[44] = -0.121 * v[2] + 0.006 * v[7] + v[13];
   y[45] = -0.121 * v[14] + 0.006 * v[9] + v[16];
   y[46] = -1.23062093848603e-12 + -0.121 * v[1] + v[5];
   y[48] = -0.206 * v[2] + 0.001 * v[12] + 0.007 * v[7] + v[13];
   y[49] = -0.206 * v[14] + 0.001 * v[15] + 0.007 * v[9] + v[16];
   y[50] = -0.206 * v[1] + -1.4357244282337e-12 + 0.001 * v[11] + v[5];
   y[52] = -0.312 * v[2] + -0.001 * v[12] + 0.006 * v[7] + v[13];
   y[53] = -0.312 * v[14] + -0.001 * v[15] + 0.006 * v[9] + v[16];
   y[54] = -0.312 * v[1] + -1.23062093848603e-12 + -0.001 * v[11] + v[5];
   y[56] = -0.057 * v[2] + 0.003 * v[12] + 0.008 * v[7] + v[13];
   y[57] = -0.057 * v[14] + 0.003 * v[15] + 0.008 * v[9] + v[16];
   y[58] = -0.057 * v[1] + -1.64082791798137e-12 + 0.003 * v[11] + v[5];
   y[60] = -0.266 * v[2] + 0.006 * v[7] + v[13];
   y[61] = -0.266 * v[14] + 0.006 * v[9] + v[16];
   y[62] = -1.23062093848603e-12 + -0.266 * v[1] + v[5];
   y[64] = -0.397 * v[2] + -0.001 * v[12] + -0.018 * v[7] + v[13];
   y[65] = -0.397 * v[14] + -0.001 * v[15] + -0.018 * v[9] + v[16];
   y[66] = -0.397 * v[1] + 3.69186281545808e-12 + -0.001 * v[11] + v[5];
   y[68] = -0.164 * v[2] + 0.007 * v[7] + v[13];
   y[69] = -0.164 * v[14] + 0.007 * v[9] + v[16];
   y[70] = -1.4357244282337e-12 + -0.164 * v[1] + v[5];
   v[17] = v[13] + -0.3922 * v[2] + 0.1333 * v[7];
   y[72] = -0.009 * v[7] + v[17];
   v[18] = v[16] + -0.3922 * v[14] + 0.1333 * v[9];
   y[73] = -0.009 * v[9] + v[18];
   v[19] = v[5] + -2.73402951833646e-11 + -0.3922 * v[1];
   y[74] = 1.84593140772904e-12 + v[19];
   y[76] = -0.052 * v[7] + v[17];
   y[77] = -0.052 * v[9] + v[18];
   y[78] = 1.06653814668789e-11 + v[19];
   v[20] = cos(x[3]);
   v[21] = sin(x[3]);
   v[22] = v[2] * v[20] + v[12] * v[21];
   v[23] = 0. - v[21];
   v[24] = v[2] * v[23] + v[12] * v[20];
   y[80] = -0.002 * v[22] + 0.027 * v[24] + -0.001 * v[7] + v[17];
   v[25] = v[14] * v[20] + v[15] * v[21];
   v[26] = v[14] * v[23] + v[15] * v[20];
   y[81] = -0.002 * v[25] + 0.027 * v[26] + -0.001 * v[9] + v[18];
   v[21] = v[1] * v[20] + v[11] * v[21];
   v[23] = v[1] * v[23] + v[11] * v[20];
   y[82] = -0.002 * v[21] + 2.05103489747671e-13 + 0.027 * v[23] + v[19];
   y[84] = 0.001 * v[22] + -0.01 * v[24] + v[17];
   y[85] = 0.001 * v[25] + -0.01 * v[26] + v[18];
   y[86] = 0.001 * v[21] + -0.01 * v[23] + v[19];
   v[20] = sin(x[4]);
   v[27] = -2.05103489747671e-10 * v[20];
   v[28] = -1. * v[20] - -2.05103489747671e-10 * v[27];
   v[29] = cos(x[4]);
   v[30] = -2.05103489747671e-10 * v[29];
   v[31] = 0. - -1. * v[29];
   v[32] = v[22] * v[28] + v[24] * v[30] + v[7] * v[31];
   v[33] = -1. * v[24] + -2.05103489747671e-10 * v[7];
   v[34] = v[17] + -0.0997 * v[24] + -2.04488118229785e-11 * v[7];
   y[88] = -0.01 * v[32] + -0.001 * v[33] + v[34];
   v[35] = v[25] * v[28] + v[26] * v[30] + v[9] * v[31];
   v[36] = -1. * v[26] + -2.05103489747671e-10 * v[9];
   v[37] = v[18] + -0.0997 * v[26] + -2.04488118229785e-11 * v[9];
   y[89] = -0.01 * v[35] + -0.001 * v[36] + v[37];
   v[31] = v[21] * v[28] + v[23] * v[30] + -2.05103489747671e-10 * v[31];
   v[30] = 4.2067441506673e-20 + -1. * v[23];
   v[28] = v[19] + 4.19412266608633e-21 + -0.0997 * v[23];
   y[90] = -0.01 * v[31] + -0.001 * v[30] + v[28];
   y[92] = 0.008 * v[32] + -0.001 * v[33] + v[34];
   y[93] = 0.008 * v[35] + -0.001 * v[36] + v[37];
   y[94] = 0.008 * v[31] + -0.001 * v[30] + v[28];
   v[38] = v[22] * v[29] + v[24] * v[27] + v[7] * v[20];
   y[96] = 0.001 * v[38] + -0.001 * v[32] + -0.036 * v[33] + v[34];
   v[39] = v[25] * v[29] + v[26] * v[27] + v[9] * v[20];
   y[97] = 0.001 * v[39] + -0.001 * v[35] + -0.036 * v[36] + v[37];
   v[29] = v[21] * v[29] + v[23] * v[27] + -2.05103489747671e-10 * v[20];
   y[98] = 0.001 * v[29] + -0.001 * v[31] + -0.036 * v[30] + v[28];
   y[100] = 0.001 * v[38] + -0.03 * v[32] + v[34];
   y[101] = 0.001 * v[39] + -0.03 * v[35] + v[37];
   y[102] = 0.001 * v[29] + -0.03 * v[31] + v[28];
   v[27] = cos(x[5]);
   v[20] = sin(x[5]);
   v[40] = v[27] + -1.22464679939853e-16 * v[20];
   v[41] = -1.22464679914735e-16 * v[27] + -2.05103489747671e-10 * v[20];
   v[20] = -1.22464679914735e-16 * v[27] + -1. * v[20];
   v[27] = v[38] * v[40] + v[32] * v[41] + v[33] * v[20];
   v[42] = v[20] - -2.05103489747671e-10 * v[41];
   v[43] = -2.05103489747671e-10 * v[40] - 1.22464679889617e-16 * v[20];
   v[44] = 1.22464679889617e-16 * v[41] - v[40];
   v[45] = v[38] * v[42] + v[32] * v[43] + v[33] * v[44];
   v[46] = 1.22464679889617e-16 * v[38] + v[32] + -2.05103489747671e-10 * v[33];
   v[47] = v[34] + 0.0996 * v[32] + -2.0428301480127e-11 * v[33];
   y[104] = 0.001 * v[27] + 0.001 * v[45] + -0.029 * v[46] + v[47];
   v[48] = v[39] * v[40] + v[35] * v[41] + v[36] * v[20];
   v[49] = v[39] * v[42] + v[35] * v[43] + v[36] * v[44];
   v[50] = 1.22464679889617e-16 * v[39] + v[35] + -2.05103489747671e-10 * v[36];
   v[51] = v[37] + 0.0996 * v[35] + -2.0428301480127e-11 * v[36];
   y[105] = 0.001 * v[48] + 0.001 * v[49] + -0.029 * v[50] + v[51];
   v[20] = v[29] * v[40] + v[31] * v[41] + v[30] * v[20];
   v[44] = v[29] * v[42] + v[31] * v[43] + v[30] * v[44];
   v[43] = 1.22464679889617e-16 * v[29] + v[31] + -2.05103489747671e-10 * v[30];
   v[42] = v[28] + 0.0996 * v[31] + -2.0428301480127e-11 * v[30];
   y[106] = 0.001 * v[20] + 0.001 * v[44] + -0.029 * v[43] + v[42];
   y[112] = -0.207500010728836 * v[6] + -0.0010048191761598 * v[0] + 0.14300000667572 * v[7];
   y[113] = -0.207500010728836 * v[8] + -0.0010048191761598 * v[10] + 0.14300000667572 * v[9];
   y[114] = 0.1625 + -0.207500010728836 * v[4] + -2.93298004031305e-11 + -0.0010048191761598 * v[3];
   y[116] = -0.193579494953156 * v[2] + 0.000576903112232685 * v[12] + 0.00775608373805881 * v[7] + v[13];
   y[117] = -0.193579494953156 * v[14] + 0.000576903112232685 * v[15] + 0.00775608373805881 * v[9] + v[16];
   y[118] = -0.193579494953156 * v[1] + -1.59079984145102e-12 + 0.000576903112232685 * v[11] + v[5];
   y[120] = -0.000469038146547973 * v[22] + 0.0070461961440742 * v[24] + -0.0242255218327045 * v[7] + v[17];
   y[121] = -0.000469038146547973 * v[25] + 0.0070461961440742 * v[26] + -0.0242255218327045 * v[9] + v[18];
   y[122] = -0.000469038146547973 * v[21] + 4.9687390688461e-12 + 0.0070461961440742 * v[23] + v[19];
   y[124] = 0.000816664425656199 * v[38] + -0.0113645261153579 * v[32] + -0.0146686462685466 * v[33] + v[34];
   y[125] = 0.000816664425656199 * v[39] + -0.0113645261153579 * v[35] + -0.0146686462685466 * v[36] + v[37];
   y[126] = 0.000816664425656199 * v[29] + -0.0113645261153579 * v[31] + -0.0146686462685466 * v[30] + v[28];
   y[128] = 0.00100000004749745 * v[27] + 0.00100000004749745 * v[45] + -0.028999999165535 * v[46] + v[47];
   y[129] = 0.00100000004749745 * v[48] + 0.00100000004749745 * v[49] + -0.028999999165535 * v[50] + v[51];
   y[130] = 0.00100000004749745 * v[20] + 0.00100000004749745 * v[44] + -0.028999999165535 * v[43] + v[42];
   // dependent variables without operations
   y[0] = 0.;
   y[1] = 0.;
   y[2] = 0.1625;
   y[3] = 0.100000001490116;
   y[7] = 0.0780000016093254;
   y[11] = 0.0769999995827675;
   y[15] = 0.061999998986721;
   y[19] = 0.0610000006854534;
   y[23] = 0.0769999995827675;
   y[27] = 0.0610000006854534;
   y[31] = 0.0700000002980232;
   y[35] = 0.0799999982118607;
   y[39] = 0.0719999969005585;
   y[43] = 0.0570000000298023;
   y[47] = 0.0570000000298023;
   y[51] = 0.0570000000298023;
   y[55] = 0.0560000017285347;
   y[59] = 0.0649999976158142;
   y[63] = 0.0570000000298023;
   y[67] = 0.0520000010728836;
   y[71] = 0.0570000000298023;
   y[75] = 0.0469999983906746;
   y[79] = 0.0469999983906746;
   y[83] = 0.0450000017881393;
   y[87] = 0.046000000089407;
   y[91] = 0.0469999983906746;
   y[95] = 0.0469999983906746;
   y[99] = 0.0469999983906746;
   y[103] = 0.0469999983906746;
   y[107] = 0.0430000014603138;
   y[108] = 0.;
   y[109] = 0.;
   y[110] = 0.1625;
   y[111] = 0.100000001490116;
   y[115] = 0.286502420902252;
   y[119] = 0.257050633430481;
   y[123] = 0.0756581649184227;
   y[127] = 0.0707167387008667;
   y[131] = 0.0430000014603138;


        Debug output;

        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[0],
                y[1],
                y[2],
                y[3]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[4],
                y[5],
                y[6],
                y[7]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[8],
                y[9],
                y[10],
                y[11]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[12],
                y[13],
                y[14],
                y[15]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[16],
                y[17],
                y[18],
                y[19]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[20],
                y[21],
                y[22],
                y[23]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[24],
                y[25],
                y[26],
                y[27]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[28],
                y[29],
                y[30],
                y[31]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[32],
                y[33],
                y[34],
                y[35]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[36],
                y[37],
                y[38],
                y[39]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[40],
                y[41],
                y[42],
                y[43]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[44],
                y[45],
                y[46],
                y[47]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[48],
                y[49],
                y[50],
                y[51]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[52],
                y[53],
                y[54],
                y[55]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[56],
                y[57],
                y[58],
                y[59]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[60],
                y[61],
                y[62],
                y[63]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[64],
                y[65],
                y[66],
                y[67]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[68],
                y[69],
                y[70],
                y[71]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[72],
                y[73],
                y[74],
                y[75]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[76],
                y[77],
                y[78],
                y[79]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[80],
                y[81],
                y[82],
                y[83]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[84],
                y[85],
                y[86],
                y[87]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[88],
                y[89],
                y[90],
                y[91]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[92],
                y[93],
                y[94],
                y[95]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[96],
                y[97],
                y[98],
                y[99]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[100],
                y[101],
                y[102],
                y[103]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[104],
                y[105],
                y[106],
                y[107]));
        

        
        
        
        
        
        

        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[36 ],
                                                         y[37 ],
                                                         y[38 ],
                                                         y[39 ]))
        {
            output.second.emplace_back(0, 9);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[40 ],
                                                         y[41 ],
                                                         y[42 ],
                                                         y[43 ]))
        {
            output.second.emplace_back(0, 10);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[44 ],
                                                         y[45 ],
                                                         y[46 ],
                                                         y[47 ]))
        {
            output.second.emplace_back(0, 11);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[48 ],
                                                         y[49 ],
                                                         y[50 ],
                                                         y[51 ]))
        {
            output.second.emplace_back(0, 12);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[52 ],
                                                         y[53 ],
                                                         y[54 ],
                                                         y[55 ]))
        {
            output.second.emplace_back(0, 13);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[56 ],
                                                         y[57 ],
                                                         y[58 ],
                                                         y[59 ]))
        {
            output.second.emplace_back(0, 14);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[60 ],
                                                         y[61 ],
                                                         y[62 ],
                                                         y[63 ]))
        {
            output.second.emplace_back(0, 15);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[64 ],
                                                         y[65 ],
                                                         y[66 ],
                                                         y[67 ]))
        {
            output.second.emplace_back(0, 16);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[68 ],
                                                         y[69 ],
                                                         y[70 ],
                                                         y[71 ]))
        {
            output.second.emplace_back(0, 17);
        }

        
        
        
        
        
        
        
        

        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(0, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(0, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(0, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(0, 21);
        }

        
        
        
        
        
        
        
        

        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(0, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(0, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(0, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(0, 25);
        }

        
        
        
        
        
        
        
        

        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                         y[1 ],
                                                         y[2 ],
                                                         y[3 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(0, 26);
        }

        
        
        
        
        
        
        
        

        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(1, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(1, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(1, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(1, 21);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(2, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(2, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(2, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(2, 21);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(3, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(3, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(3, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(3, 21);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(4, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(4, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(4, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(4, 21);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(5, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(5, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(5, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(5, 21);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(6, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(6, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(6, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(6, 21);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(7, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(7, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(7, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(7, 21);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[72 ],
                                                         y[73 ],
                                                         y[74 ],
                                                         y[75 ]))
        {
            output.second.emplace_back(8, 18);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[76 ],
                                                         y[77 ],
                                                         y[78 ],
                                                         y[79 ]))
        {
            output.second.emplace_back(8, 19);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[80 ],
                                                         y[81 ],
                                                         y[82 ],
                                                         y[83 ]))
        {
            output.second.emplace_back(8, 20);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[84 ],
                                                         y[85 ],
                                                         y[86 ],
                                                         y[87 ]))
        {
            output.second.emplace_back(8, 21);
        }

        
        
        
        
        
        
        
        

        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(1, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(1, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(1, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(1, 25);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(2, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(2, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(2, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(2, 25);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(3, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(3, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(3, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(3, 25);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(4, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(4, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(4, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(4, 25);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(5, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(5, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(5, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(5, 25);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(6, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(6, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(6, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(6, 25);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(7, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(7, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(7, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(7, 25);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[88 ],
                                                         y[89 ],
                                                         y[90 ],
                                                         y[91 ]))
        {
            output.second.emplace_back(8, 22);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[92 ],
                                                         y[93 ],
                                                         y[94 ],
                                                         y[95 ]))
        {
            output.second.emplace_back(8, 23);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[96 ],
                                                         y[97 ],
                                                         y[98 ],
                                                         y[99 ]))
        {
            output.second.emplace_back(8, 24);
        }

        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[100 ],
                                                         y[101 ],
                                                         y[102 ],
                                                         y[103 ]))
        {
            output.second.emplace_back(8, 25);
        }

        
        
        
        
        
        
        
        

        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                         y[5 ],
                                                         y[6 ],
                                                         y[7 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(1, 26);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                         y[9 ],
                                                         y[10 ],
                                                         y[11 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(2, 26);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                         y[13 ],
                                                         y[14 ],
                                                         y[15 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(3, 26);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                         y[17 ],
                                                         y[18 ],
                                                         y[19 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(4, 26);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                         y[21 ],
                                                         y[22 ],
                                                         y[23 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(5, 26);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                         y[25 ],
                                                         y[26 ],
                                                         y[27 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(6, 26);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                         y[29 ],
                                                         y[30 ],
                                                         y[31 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(7, 26);
        }

        
        
        

        
        

        if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                         y[33 ],
                                                         y[34 ],
                                                         y[35 ],
                                                         y[104 ],
                                                         y[105 ],
                                                         y[106 ],
                                                         y[107 ]))
        {
            output.second.emplace_back(8, 26);
        }

        
        
        

        return output;
    }

    template <std::size_t rake>
        static inline bool fkcc(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 52> v;
        std::array<FloatVector<rake, 1>, 132> y;

           v[0] = cos(x[0]);
   v[1] = sin(x[0]);
   v[2] = -1. * v[0] + -1.22464679914735e-16 * v[1];
   v[3] = cos(x[1]);
   v[1] = 1.22464679914735e-16 * v[0] + -1. * v[1];
   v[0] = 0. - v[1];
   v[4] = sin(x[1]);
   v[5] = -2.05103489747671e-10 * v[4];
   v[6] = v[2] * v[3] + v[0] * v[5];
   v[7] = -1. * v[0];
   y[4] = -0.416 * v[6] + 0.143 * v[7];
   v[8] = v[1] * v[3] + v[2] * v[5];
   v[9] = -1. * v[2];
   y[5] = -0.416 * v[8] + 0.143 * v[9];
   y[6] = 0.1625 + -2.9329799033917e-11 + -0.416 * v[4];
   y[8] = -0.015 * v[6] + 0.134 * v[7];
   y[9] = -0.015 * v[8] + 0.134 * v[9];
   y[10] = 0.1625 + -2.74838676261879e-11 + -0.015 * v[4];
   y[12] = -0.14 * v[6] + 0.138 * v[7];
   y[13] = -0.14 * v[8] + 0.138 * v[9];
   y[14] = 0.1625 + -2.83042815851786e-11 + -0.14 * v[4];
   v[5] = -1. * v[4] - -2.05103489747671e-10 * v[5];
   v[10] = -2.05103489747671e-10 * v[3];
   v[0] = v[2] * v[5] + v[0] * v[10];
   y[16] = -0.285 * v[6] + -0.001 * v[0] + 0.139 * v[7];
   v[10] = v[1] * v[5] + v[2] * v[10];
   y[17] = -0.285 * v[8] + -0.001 * v[10] + 0.139 * v[9];
   v[3] = 0. - -1. * v[3];
   y[18] = 0.1625 + -0.285 * v[4] + -2.85093850749263e-11 + -0.001 * v[3];
   y[20] = -0.376 * v[6] + 0.001 * v[0] + 0.138 * v[7];
   y[21] = -0.376 * v[8] + 0.001 * v[10] + 0.138 * v[9];
   y[22] = 0.1625 + -0.376 * v[4] + -2.83042815851786e-11 + 0.001 * v[3];
   y[24] = -0.222 * v[6] + 0.001 * v[0] + 0.139 * v[7];
   y[25] = -0.222 * v[8] + 0.001 * v[10] + 0.139 * v[9];
   y[26] = 0.1625 + -0.222 * v[4] + -2.85093850749263e-11 + 0.001 * v[3];
   y[28] = -0.055 * v[6] + 0.008 * v[0] + 0.14 * v[7];
   y[29] = -0.055 * v[8] + 0.008 * v[10] + 0.14 * v[9];
   y[30] = 0.1625 + -0.055 * v[4] + -2.8714488564674e-11 + 0.008 * v[3];
   y[32] = -0.001 * v[6] + -0.002 * v[0] + 0.143 * v[7];
   y[33] = -0.001 * v[8] + -0.002 * v[10] + 0.143 * v[9];
   y[34] = 0.1625 + -0.001 * v[4] + -2.9329799033917e-11 + -0.002 * v[3];
   v[5] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = v[6] * v[5] + v[0] * v[1];
   v[11] = 0. - v[1];
   v[12] = v[6] * v[11] + v[0] * v[5];
   v[13] = -0.425 * v[6];
   y[36] = -0.01 * v[2] + 0.002 * v[12] + 0.031 * v[7] + v[13];
   v[14] = v[8] * v[5] + v[10] * v[1];
   v[15] = v[8] * v[11] + v[10] * v[5];
   v[16] = -0.425 * v[8];
   y[37] = -0.01 * v[14] + 0.002 * v[15] + 0.031 * v[9] + v[16];
   v[1] = v[4] * v[5] + v[3] * v[1];
   v[11] = v[4] * v[11] + v[3] * v[5];
   v[5] = 0.1625 + -0.425 * v[4];
   y[38] = -0.01 * v[1] + -6.3582081821778e-12 + 0.002 * v[11] + v[5];
   y[40] = -0.387 * v[2] + 0.014 * v[7] + v[13];
   y[41] = -0.387 * v[14] + 0.014 * v[9] + v[16];
   y[42] = -2.8714488564674e-12 + -0.387 * v[1] + v[5];
   y[44] = -0.121 * v[2] + 0.006 * v[7] + v[13];
   y[45] = -0.121 * v[14] + 0.006 * v[9] + v[16];
   y[46] = -1.23062093848603e-12 + -0.121 * v[1] + v[5];
   y[48] = -0.206 * v[2] + 0.001 * v[12] + 0.007 * v[7] + v[13];
   y[49] = -0.206 * v[14] + 0.001 * v[15] + 0.007 * v[9] + v[16];
   y[50] = -0.206 * v[1] + -1.4357244282337e-12 + 0.001 * v[11] + v[5];
   y[52] = -0.312 * v[2] + -0.001 * v[12] + 0.006 * v[7] + v[13];
   y[53] = -0.312 * v[14] + -0.001 * v[15] + 0.006 * v[9] + v[16];
   y[54] = -0.312 * v[1] + -1.23062093848603e-12 + -0.001 * v[11] + v[5];
   y[56] = -0.057 * v[2] + 0.003 * v[12] + 0.008 * v[7] + v[13];
   y[57] = -0.057 * v[14] + 0.003 * v[15] + 0.008 * v[9] + v[16];
   y[58] = -0.057 * v[1] + -1.64082791798137e-12 + 0.003 * v[11] + v[5];
   y[60] = -0.266 * v[2] + 0.006 * v[7] + v[13];
   y[61] = -0.266 * v[14] + 0.006 * v[9] + v[16];
   y[62] = -1.23062093848603e-12 + -0.266 * v[1] + v[5];
   y[64] = -0.397 * v[2] + -0.001 * v[12] + -0.018 * v[7] + v[13];
   y[65] = -0.397 * v[14] + -0.001 * v[15] + -0.018 * v[9] + v[16];
   y[66] = -0.397 * v[1] + 3.69186281545808e-12 + -0.001 * v[11] + v[5];
   y[68] = -0.164 * v[2] + 0.007 * v[7] + v[13];
   y[69] = -0.164 * v[14] + 0.007 * v[9] + v[16];
   y[70] = -1.4357244282337e-12 + -0.164 * v[1] + v[5];
   v[17] = v[13] + -0.3922 * v[2] + 0.1333 * v[7];
   y[72] = -0.009 * v[7] + v[17];
   v[18] = v[16] + -0.3922 * v[14] + 0.1333 * v[9];
   y[73] = -0.009 * v[9] + v[18];
   v[19] = v[5] + -2.73402951833646e-11 + -0.3922 * v[1];
   y[74] = 1.84593140772904e-12 + v[19];
   y[76] = -0.052 * v[7] + v[17];
   y[77] = -0.052 * v[9] + v[18];
   y[78] = 1.06653814668789e-11 + v[19];
   v[20] = cos(x[3]);
   v[21] = sin(x[3]);
   v[22] = v[2] * v[20] + v[12] * v[21];
   v[23] = 0. - v[21];
   v[24] = v[2] * v[23] + v[12] * v[20];
   y[80] = -0.002 * v[22] + 0.027 * v[24] + -0.001 * v[7] + v[17];
   v[25] = v[14] * v[20] + v[15] * v[21];
   v[26] = v[14] * v[23] + v[15] * v[20];
   y[81] = -0.002 * v[25] + 0.027 * v[26] + -0.001 * v[9] + v[18];
   v[21] = v[1] * v[20] + v[11] * v[21];
   v[23] = v[1] * v[23] + v[11] * v[20];
   y[82] = -0.002 * v[21] + 2.05103489747671e-13 + 0.027 * v[23] + v[19];
   y[84] = 0.001 * v[22] + -0.01 * v[24] + v[17];
   y[85] = 0.001 * v[25] + -0.01 * v[26] + v[18];
   y[86] = 0.001 * v[21] + -0.01 * v[23] + v[19];
   v[20] = sin(x[4]);
   v[27] = -2.05103489747671e-10 * v[20];
   v[28] = -1. * v[20] - -2.05103489747671e-10 * v[27];
   v[29] = cos(x[4]);
   v[30] = -2.05103489747671e-10 * v[29];
   v[31] = 0. - -1. * v[29];
   v[32] = v[22] * v[28] + v[24] * v[30] + v[7] * v[31];
   v[33] = -1. * v[24] + -2.05103489747671e-10 * v[7];
   v[34] = v[17] + -0.0997 * v[24] + -2.04488118229785e-11 * v[7];
   y[88] = -0.01 * v[32] + -0.001 * v[33] + v[34];
   v[35] = v[25] * v[28] + v[26] * v[30] + v[9] * v[31];
   v[36] = -1. * v[26] + -2.05103489747671e-10 * v[9];
   v[37] = v[18] + -0.0997 * v[26] + -2.04488118229785e-11 * v[9];
   y[89] = -0.01 * v[35] + -0.001 * v[36] + v[37];
   v[31] = v[21] * v[28] + v[23] * v[30] + -2.05103489747671e-10 * v[31];
   v[30] = 4.2067441506673e-20 + -1. * v[23];
   v[28] = v[19] + 4.19412266608633e-21 + -0.0997 * v[23];
   y[90] = -0.01 * v[31] + -0.001 * v[30] + v[28];
   y[92] = 0.008 * v[32] + -0.001 * v[33] + v[34];
   y[93] = 0.008 * v[35] + -0.001 * v[36] + v[37];
   y[94] = 0.008 * v[31] + -0.001 * v[30] + v[28];
   v[38] = v[22] * v[29] + v[24] * v[27] + v[7] * v[20];
   y[96] = 0.001 * v[38] + -0.001 * v[32] + -0.036 * v[33] + v[34];
   v[39] = v[25] * v[29] + v[26] * v[27] + v[9] * v[20];
   y[97] = 0.001 * v[39] + -0.001 * v[35] + -0.036 * v[36] + v[37];
   v[29] = v[21] * v[29] + v[23] * v[27] + -2.05103489747671e-10 * v[20];
   y[98] = 0.001 * v[29] + -0.001 * v[31] + -0.036 * v[30] + v[28];
   y[100] = 0.001 * v[38] + -0.03 * v[32] + v[34];
   y[101] = 0.001 * v[39] + -0.03 * v[35] + v[37];
   y[102] = 0.001 * v[29] + -0.03 * v[31] + v[28];
   v[27] = cos(x[5]);
   v[20] = sin(x[5]);
   v[40] = v[27] + -1.22464679939853e-16 * v[20];
   v[41] = -1.22464679914735e-16 * v[27] + -2.05103489747671e-10 * v[20];
   v[20] = -1.22464679914735e-16 * v[27] + -1. * v[20];
   v[27] = v[38] * v[40] + v[32] * v[41] + v[33] * v[20];
   v[42] = v[20] - -2.05103489747671e-10 * v[41];
   v[43] = -2.05103489747671e-10 * v[40] - 1.22464679889617e-16 * v[20];
   v[44] = 1.22464679889617e-16 * v[41] - v[40];
   v[45] = v[38] * v[42] + v[32] * v[43] + v[33] * v[44];
   v[46] = 1.22464679889617e-16 * v[38] + v[32] + -2.05103489747671e-10 * v[33];
   v[47] = v[34] + 0.0996 * v[32] + -2.0428301480127e-11 * v[33];
   y[104] = 0.001 * v[27] + 0.001 * v[45] + -0.029 * v[46] + v[47];
   v[48] = v[39] * v[40] + v[35] * v[41] + v[36] * v[20];
   v[49] = v[39] * v[42] + v[35] * v[43] + v[36] * v[44];
   v[50] = 1.22464679889617e-16 * v[39] + v[35] + -2.05103489747671e-10 * v[36];
   v[51] = v[37] + 0.0996 * v[35] + -2.0428301480127e-11 * v[36];
   y[105] = 0.001 * v[48] + 0.001 * v[49] + -0.029 * v[50] + v[51];
   v[20] = v[29] * v[40] + v[31] * v[41] + v[30] * v[20];
   v[44] = v[29] * v[42] + v[31] * v[43] + v[30] * v[44];
   v[43] = 1.22464679889617e-16 * v[29] + v[31] + -2.05103489747671e-10 * v[30];
   v[42] = v[28] + 0.0996 * v[31] + -2.0428301480127e-11 * v[30];
   y[106] = 0.001 * v[20] + 0.001 * v[44] + -0.029 * v[43] + v[42];
   y[112] = -0.207500010728836 * v[6] + -0.0010048191761598 * v[0] + 0.14300000667572 * v[7];
   y[113] = -0.207500010728836 * v[8] + -0.0010048191761598 * v[10] + 0.14300000667572 * v[9];
   y[114] = 0.1625 + -0.207500010728836 * v[4] + -2.93298004031305e-11 + -0.0010048191761598 * v[3];
   y[116] = -0.193579494953156 * v[2] + 0.000576903112232685 * v[12] + 0.00775608373805881 * v[7] + v[13];
   y[117] = -0.193579494953156 * v[14] + 0.000576903112232685 * v[15] + 0.00775608373805881 * v[9] + v[16];
   y[118] = -0.193579494953156 * v[1] + -1.59079984145102e-12 + 0.000576903112232685 * v[11] + v[5];
   y[120] = -0.000469038146547973 * v[22] + 0.0070461961440742 * v[24] + -0.0242255218327045 * v[7] + v[17];
   y[121] = -0.000469038146547973 * v[25] + 0.0070461961440742 * v[26] + -0.0242255218327045 * v[9] + v[18];
   y[122] = -0.000469038146547973 * v[21] + 4.9687390688461e-12 + 0.0070461961440742 * v[23] + v[19];
   y[124] = 0.000816664425656199 * v[38] + -0.0113645261153579 * v[32] + -0.0146686462685466 * v[33] + v[34];
   y[125] = 0.000816664425656199 * v[39] + -0.0113645261153579 * v[35] + -0.0146686462685466 * v[36] + v[37];
   y[126] = 0.000816664425656199 * v[29] + -0.0113645261153579 * v[31] + -0.0146686462685466 * v[30] + v[28];
   y[128] = 0.00100000004749745 * v[27] + 0.00100000004749745 * v[45] + -0.028999999165535 * v[46] + v[47];
   y[129] = 0.00100000004749745 * v[48] + 0.00100000004749745 * v[49] + -0.028999999165535 * v[50] + v[51];
   y[130] = 0.00100000004749745 * v[20] + 0.00100000004749745 * v[44] + -0.028999999165535 * v[43] + v[42];
   // dependent variables without operations
   y[0] = 0.;
   y[1] = 0.;
   y[2] = 0.1625;
   y[3] = 0.100000001490116;
   y[7] = 0.0780000016093254;
   y[11] = 0.0769999995827675;
   y[15] = 0.061999998986721;
   y[19] = 0.0610000006854534;
   y[23] = 0.0769999995827675;
   y[27] = 0.0610000006854534;
   y[31] = 0.0700000002980232;
   y[35] = 0.0799999982118607;
   y[39] = 0.0719999969005585;
   y[43] = 0.0570000000298023;
   y[47] = 0.0570000000298023;
   y[51] = 0.0570000000298023;
   y[55] = 0.0560000017285347;
   y[59] = 0.0649999976158142;
   y[63] = 0.0570000000298023;
   y[67] = 0.0520000010728836;
   y[71] = 0.0570000000298023;
   y[75] = 0.0469999983906746;
   y[79] = 0.0469999983906746;
   y[83] = 0.0450000017881393;
   y[87] = 0.046000000089407;
   y[91] = 0.0469999983906746;
   y[95] = 0.0469999983906746;
   y[99] = 0.0469999983906746;
   y[103] = 0.0469999983906746;
   y[107] = 0.0430000014603138;
   y[108] = 0.;
   y[109] = 0.;
   y[110] = 0.1625;
   y[111] = 0.100000001490116;
   y[115] = 0.286502420902252;
   y[119] = 0.257050633430481;
   y[123] = 0.0756581649184227;
   y[127] = 0.0707167387008667;
   y[131] = 0.0430000014603138;

        





//
// environment vs. robot collisions
//

// wrist_3_link
if (sphere_environment_in_collision(environment,
                                    y[128],
                                    y[129],
                                    y[130],
                                    y[131])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[104],
                                        y[105],
                                        y[106],
                                        y[107])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// wrist_2_link
if (sphere_environment_in_collision(environment,
                                    y[124],
                                    y[125],
                                    y[126],
                                    y[127])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[88],
                                        y[89],
                                        y[90],
                                        y[91])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[92],
                                        y[93],
                                        y[94],
                                        y[95])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[96],
                                        y[97],
                                        y[98],
                                        y[99])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[100],
                                        y[101],
                                        y[102],
                                        y[103])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// wrist_1_link
if (sphere_environment_in_collision(environment,
                                    y[120],
                                    y[121],
                                    y[122],
                                    y[123])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[72],
                                        y[73],
                                        y[74],
                                        y[75])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[76],
                                        y[77],
                                        y[78],
                                        y[79])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[80],
                                        y[81],
                                        y[82],
                                        y[83])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[84],
                                        y[85],
                                        y[86],
                                        y[87])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// forearm_link
if (sphere_environment_in_collision(environment,
                                    y[116],
                                    y[117],
                                    y[118],
                                    y[119])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[36],
                                        y[37],
                                        y[38],
                                        y[39])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[40],
                                        y[41],
                                        y[42],
                                        y[43])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[44],
                                        y[45],
                                        y[46],
                                        y[47])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[48],
                                        y[49],
                                        y[50],
                                        y[51])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[52],
                                        y[53],
                                        y[54],
                                        y[55])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[56],
                                        y[57],
                                        y[58],
                                        y[59])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[60],
                                        y[61],
                                        y[62],
                                        y[63])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[64],
                                        y[65],
                                        y[66],
                                        y[67])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[68],
                                        y[69],
                                        y[70],
                                        y[71])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// upper_arm_link
if (sphere_environment_in_collision(environment,
                                    y[112],
                                    y[113],
                                    y[114],
                                    y[115])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[4],
                                        y[5],
                                        y[6],
                                        y[7])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[8],
                                        y[9],
                                        y[10],
                                        y[11])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[12],
                                        y[13],
                                        y[14],
                                        y[15])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[16],
                                        y[17],
                                        y[18],
                                        y[19])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[20],
                                        y[21],
                                        y[22],
                                        y[23])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[24],
                                        y[25],
                                        y[26],
                                        y[27])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[28],
                                        y[29],
                                        y[30],
                                        y[31])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[32],
                                        y[33],
                                        y[34],
                                        y[35])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// shoulder_link
if (sphere_environment_in_collision(environment,
                                    y[108],
                                    y[109],
                                    y[110],
                                    y[111])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3])) [[unlikely]]
    {
        return false;
    }
    
}



//
// robot self-collisions
//












// shoulder_link vs. forearm_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[108],
                                                 y[109],
                                                 y[110],
                                                 y[111],
                                                 y[116],
                                                 y[117],
                                                 y[118],
                                                 y[119])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[36 ],
                                                     y[37 ],
                                                     y[38 ],
                                                     y[39 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[40 ],
                                                     y[41 ],
                                                     y[42 ],
                                                     y[43 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[44 ],
                                                     y[45 ],
                                                     y[46 ],
                                                     y[47 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[48 ],
                                                     y[49 ],
                                                     y[50 ],
                                                     y[51 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[52 ],
                                                     y[53 ],
                                                     y[54 ],
                                                     y[55 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[56 ],
                                                     y[57 ],
                                                     y[58 ],
                                                     y[59 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[60 ],
                                                     y[61 ],
                                                     y[62 ],
                                                     y[63 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[64 ],
                                                     y[65 ],
                                                     y[66 ],
                                                     y[67 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[68 ],
                                                     y[69 ],
                                                     y[70 ],
                                                     y[71 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// shoulder_link vs. wrist_1_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[108],
                                                 y[109],
                                                 y[110],
                                                 y[111],
                                                 y[120],
                                                 y[121],
                                                 y[122],
                                                 y[123])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// shoulder_link vs. wrist_2_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[108],
                                                 y[109],
                                                 y[110],
                                                 y[111],
                                                 y[124],
                                                 y[125],
                                                 y[126],
                                                 y[127])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// shoulder_link vs. wrist_3_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[108],
                                                 y[109],
                                                 y[110],
                                                 y[111],
                                                 y[128],
                                                 y[129],
                                                 y[130],
                                                 y[131])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// upper_arm_link vs. wrist_1_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[112],
                                                 y[113],
                                                 y[114],
                                                 y[115],
                                                 y[120],
                                                 y[121],
                                                 y[122],
                                                 y[123])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// upper_arm_link vs. wrist_2_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[112],
                                                 y[113],
                                                 y[114],
                                                 y[115],
                                                 y[124],
                                                 y[125],
                                                 y[126],
                                                 y[127])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// upper_arm_link vs. wrist_3_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[112],
                                                 y[113],
                                                 y[114],
                                                 y[115],
                                                 y[128],
                                                 y[129],
                                                 y[130],
                                                 y[131])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
}




        return true;
    }

    template <std::size_t rake>
    static inline bool fkcc_attach(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 50> v;
        std::array<FloatVector<rake, 1>, 144> y;

           v[0] = cos(x[0]);
   v[1] = sin(x[0]);
   v[2] = -1. * v[0] + -1.22464679914735e-16 * v[1];
   v[3] = cos(x[1]);
   v[1] = 1.22464679914735e-16 * v[0] + -1. * v[1];
   v[0] = 0. - v[1];
   v[4] = sin(x[1]);
   v[5] = -2.05103489747671e-10 * v[4];
   v[6] = v[2] * v[3] + v[0] * v[5];
   v[7] = -1. * v[0];
   y[4] = -0.416 * v[6] + 0.143 * v[7];
   v[8] = v[1] * v[3] + v[2] * v[5];
   v[9] = -1. * v[2];
   y[5] = -0.416 * v[8] + 0.143 * v[9];
   y[6] = 0.1625 + -2.9329799033917e-11 + -0.416 * v[4];
   y[8] = -0.015 * v[6] + 0.134 * v[7];
   y[9] = -0.015 * v[8] + 0.134 * v[9];
   y[10] = 0.1625 + -2.74838676261879e-11 + -0.015 * v[4];
   y[12] = -0.14 * v[6] + 0.138 * v[7];
   y[13] = -0.14 * v[8] + 0.138 * v[9];
   y[14] = 0.1625 + -2.83042815851786e-11 + -0.14 * v[4];
   v[5] = -1. * v[4] - -2.05103489747671e-10 * v[5];
   v[10] = -2.05103489747671e-10 * v[3];
   v[0] = v[2] * v[5] + v[0] * v[10];
   y[16] = -0.285 * v[6] + -0.001 * v[0] + 0.139 * v[7];
   v[10] = v[1] * v[5] + v[2] * v[10];
   y[17] = -0.285 * v[8] + -0.001 * v[10] + 0.139 * v[9];
   v[3] = 0. - -1. * v[3];
   y[18] = 0.1625 + -0.285 * v[4] + -2.85093850749263e-11 + -0.001 * v[3];
   y[20] = -0.376 * v[6] + 0.001 * v[0] + 0.138 * v[7];
   y[21] = -0.376 * v[8] + 0.001 * v[10] + 0.138 * v[9];
   y[22] = 0.1625 + -0.376 * v[4] + -2.83042815851786e-11 + 0.001 * v[3];
   y[24] = -0.222 * v[6] + 0.001 * v[0] + 0.139 * v[7];
   y[25] = -0.222 * v[8] + 0.001 * v[10] + 0.139 * v[9];
   y[26] = 0.1625 + -0.222 * v[4] + -2.85093850749263e-11 + 0.001 * v[3];
   y[28] = -0.055 * v[6] + 0.008 * v[0] + 0.14 * v[7];
   y[29] = -0.055 * v[8] + 0.008 * v[10] + 0.14 * v[9];
   y[30] = 0.1625 + -0.055 * v[4] + -2.8714488564674e-11 + 0.008 * v[3];
   y[32] = -0.001 * v[6] + -0.002 * v[0] + 0.143 * v[7];
   y[33] = -0.001 * v[8] + -0.002 * v[10] + 0.143 * v[9];
   y[34] = 0.1625 + -0.001 * v[4] + -2.9329799033917e-11 + -0.002 * v[3];
   v[5] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = v[6] * v[5] + v[0] * v[1];
   v[11] = 0. - v[1];
   v[12] = v[6] * v[11] + v[0] * v[5];
   v[13] = -0.425 * v[6];
   y[36] = -0.01 * v[2] + 0.002 * v[12] + 0.031 * v[7] + v[13];
   v[14] = v[8] * v[5] + v[10] * v[1];
   v[15] = v[8] * v[11] + v[10] * v[5];
   v[16] = -0.425 * v[8];
   y[37] = -0.01 * v[14] + 0.002 * v[15] + 0.031 * v[9] + v[16];
   v[1] = v[4] * v[5] + v[3] * v[1];
   v[11] = v[4] * v[11] + v[3] * v[5];
   v[5] = 0.1625 + -0.425 * v[4];
   y[38] = -0.01 * v[1] + -6.3582081821778e-12 + 0.002 * v[11] + v[5];
   y[40] = -0.387 * v[2] + 0.014 * v[7] + v[13];
   y[41] = -0.387 * v[14] + 0.014 * v[9] + v[16];
   y[42] = -2.8714488564674e-12 + -0.387 * v[1] + v[5];
   y[44] = -0.121 * v[2] + 0.006 * v[7] + v[13];
   y[45] = -0.121 * v[14] + 0.006 * v[9] + v[16];
   y[46] = -1.23062093848603e-12 + -0.121 * v[1] + v[5];
   y[48] = -0.206 * v[2] + 0.001 * v[12] + 0.007 * v[7] + v[13];
   y[49] = -0.206 * v[14] + 0.001 * v[15] + 0.007 * v[9] + v[16];
   y[50] = -0.206 * v[1] + -1.4357244282337e-12 + 0.001 * v[11] + v[5];
   y[52] = -0.312 * v[2] + -0.001 * v[12] + 0.006 * v[7] + v[13];
   y[53] = -0.312 * v[14] + -0.001 * v[15] + 0.006 * v[9] + v[16];
   y[54] = -0.312 * v[1] + -1.23062093848603e-12 + -0.001 * v[11] + v[5];
   y[56] = -0.057 * v[2] + 0.003 * v[12] + 0.008 * v[7] + v[13];
   y[57] = -0.057 * v[14] + 0.003 * v[15] + 0.008 * v[9] + v[16];
   y[58] = -0.057 * v[1] + -1.64082791798137e-12 + 0.003 * v[11] + v[5];
   y[60] = -0.266 * v[2] + 0.006 * v[7] + v[13];
   y[61] = -0.266 * v[14] + 0.006 * v[9] + v[16];
   y[62] = -1.23062093848603e-12 + -0.266 * v[1] + v[5];
   y[64] = -0.397 * v[2] + -0.001 * v[12] + -0.018 * v[7] + v[13];
   y[65] = -0.397 * v[14] + -0.001 * v[15] + -0.018 * v[9] + v[16];
   y[66] = -0.397 * v[1] + 3.69186281545808e-12 + -0.001 * v[11] + v[5];
   y[68] = -0.164 * v[2] + 0.007 * v[7] + v[13];
   y[69] = -0.164 * v[14] + 0.007 * v[9] + v[16];
   y[70] = -1.4357244282337e-12 + -0.164 * v[1] + v[5];
   v[17] = v[13] + -0.3922 * v[2] + 0.1333 * v[7];
   y[72] = -0.009 * v[7] + v[17];
   v[18] = v[16] + -0.3922 * v[14] + 0.1333 * v[9];
   y[73] = -0.009 * v[9] + v[18];
   v[19] = v[5] + -2.73402951833646e-11 + -0.3922 * v[1];
   y[74] = 1.84593140772904e-12 + v[19];
   y[76] = -0.052 * v[7] + v[17];
   y[77] = -0.052 * v[9] + v[18];
   y[78] = 1.06653814668789e-11 + v[19];
   v[20] = cos(x[3]);
   v[21] = sin(x[3]);
   v[22] = v[2] * v[20] + v[12] * v[21];
   v[23] = 0. - v[21];
   v[24] = v[2] * v[23] + v[12] * v[20];
   y[80] = -0.002 * v[22] + 0.027 * v[24] + -0.001 * v[7] + v[17];
   v[25] = v[14] * v[20] + v[15] * v[21];
   v[26] = v[14] * v[23] + v[15] * v[20];
   y[81] = -0.002 * v[25] + 0.027 * v[26] + -0.001 * v[9] + v[18];
   v[21] = v[1] * v[20] + v[11] * v[21];
   v[23] = v[1] * v[23] + v[11] * v[20];
   y[82] = -0.002 * v[21] + 2.05103489747671e-13 + 0.027 * v[23] + v[19];
   y[84] = 0.001 * v[22] + -0.01 * v[24] + v[17];
   y[85] = 0.001 * v[25] + -0.01 * v[26] + v[18];
   y[86] = 0.001 * v[21] + -0.01 * v[23] + v[19];
   v[20] = sin(x[4]);
   v[27] = -2.05103489747671e-10 * v[20];
   v[28] = -1. * v[20] - -2.05103489747671e-10 * v[27];
   v[29] = cos(x[4]);
   v[30] = -2.05103489747671e-10 * v[29];
   v[31] = 0. - -1. * v[29];
   v[32] = v[22] * v[28] + v[24] * v[30] + v[7] * v[31];
   v[33] = -1. * v[24] + -2.05103489747671e-10 * v[7];
   v[34] = v[17] + -0.0997 * v[24] + -2.04488118229785e-11 * v[7];
   y[88] = -0.01 * v[32] + -0.001 * v[33] + v[34];
   v[35] = v[25] * v[28] + v[26] * v[30] + v[9] * v[31];
   v[36] = -1. * v[26] + -2.05103489747671e-10 * v[9];
   v[37] = v[18] + -0.0997 * v[26] + -2.04488118229785e-11 * v[9];
   y[89] = -0.01 * v[35] + -0.001 * v[36] + v[37];
   v[31] = v[21] * v[28] + v[23] * v[30] + -2.05103489747671e-10 * v[31];
   v[30] = 4.2067441506673e-20 + -1. * v[23];
   v[28] = v[19] + 4.19412266608633e-21 + -0.0997 * v[23];
   y[90] = -0.01 * v[31] + -0.001 * v[30] + v[28];
   y[92] = 0.008 * v[32] + -0.001 * v[33] + v[34];
   y[93] = 0.008 * v[35] + -0.001 * v[36] + v[37];
   y[94] = 0.008 * v[31] + -0.001 * v[30] + v[28];
   v[38] = v[22] * v[29] + v[24] * v[27] + v[7] * v[20];
   y[96] = 0.001 * v[38] + -0.001 * v[32] + -0.036 * v[33] + v[34];
   v[39] = v[25] * v[29] + v[26] * v[27] + v[9] * v[20];
   y[97] = 0.001 * v[39] + -0.001 * v[35] + -0.036 * v[36] + v[37];
   v[29] = v[21] * v[29] + v[23] * v[27] + -2.05103489747671e-10 * v[20];
   y[98] = 0.001 * v[29] + -0.001 * v[31] + -0.036 * v[30] + v[28];
   y[100] = 0.001 * v[38] + -0.03 * v[32] + v[34];
   y[101] = 0.001 * v[39] + -0.03 * v[35] + v[37];
   y[102] = 0.001 * v[29] + -0.03 * v[31] + v[28];
   v[27] = cos(x[5]);
   v[20] = sin(x[5]);
   v[40] = v[27] + -1.22464679939853e-16 * v[20];
   v[41] = -1.22464679914735e-16 * v[27] + -2.05103489747671e-10 * v[20];
   v[20] = -1.22464679914735e-16 * v[27] + -1. * v[20];
   v[27] = v[38] * v[40] + v[32] * v[41] + v[33] * v[20];
   v[42] = v[20] - -2.05103489747671e-10 * v[41];
   v[43] = -2.05103489747671e-10 * v[40] - 1.22464679889617e-16 * v[20];
   v[44] = 1.22464679889617e-16 * v[41] - v[40];
   v[45] = v[38] * v[42] + v[32] * v[43] + v[33] * v[44];
   v[46] = 1.22464679889617e-16 * v[38] + v[32] + -2.05103489747671e-10 * v[33];
   y[132] = v[34] + 0.0996 * v[32] + -2.0428301480127e-11 * v[33];
   y[104] = 0.001 * v[27] + 0.001 * v[45] + -0.029 * v[46] + y[132];
   v[47] = v[39] * v[40] + v[35] * v[41] + v[36] * v[20];
   v[48] = v[39] * v[42] + v[35] * v[43] + v[36] * v[44];
   v[49] = 1.22464679889617e-16 * v[39] + v[35] + -2.05103489747671e-10 * v[36];
   y[133] = v[37] + 0.0996 * v[35] + -2.0428301480127e-11 * v[36];
   y[105] = 0.001 * v[47] + 0.001 * v[48] + -0.029 * v[49] + y[133];
   v[20] = v[29] * v[40] + v[31] * v[41] + v[30] * v[20];
   v[44] = v[29] * v[42] + v[31] * v[43] + v[30] * v[44];
   v[43] = 1.22464679889617e-16 * v[29] + v[31] + -2.05103489747671e-10 * v[30];
   y[134] = v[28] + 0.0996 * v[31] + -2.0428301480127e-11 * v[30];
   y[106] = 0.001 * v[20] + 0.001 * v[44] + -0.029 * v[43] + y[134];
   y[112] = -0.207500010728836 * v[6] + -0.0010048191761598 * v[0] + 0.14300000667572 * v[7];
   y[113] = -0.207500010728836 * v[8] + -0.0010048191761598 * v[10] + 0.14300000667572 * v[9];
   y[114] = 0.1625 + -0.207500010728836 * v[4] + -2.93298004031305e-11 + -0.0010048191761598 * v[3];
   y[116] = -0.193579494953156 * v[2] + 0.000576903112232685 * v[12] + 0.00775608373805881 * v[7] + v[13];
   y[117] = -0.193579494953156 * v[14] + 0.000576903112232685 * v[15] + 0.00775608373805881 * v[9] + v[16];
   y[118] = -0.193579494953156 * v[1] + -1.59079984145102e-12 + 0.000576903112232685 * v[11] + v[5];
   y[120] = -0.000469038146547973 * v[22] + 0.0070461961440742 * v[24] + -0.0242255218327045 * v[7] + v[17];
   y[121] = -0.000469038146547973 * v[25] + 0.0070461961440742 * v[26] + -0.0242255218327045 * v[9] + v[18];
   y[122] = -0.000469038146547973 * v[21] + 4.9687390688461e-12 + 0.0070461961440742 * v[23] + v[19];
   y[124] = 0.000816664425656199 * v[38] + -0.0113645261153579 * v[32] + -0.0146686462685466 * v[33] + v[34];
   y[125] = 0.000816664425656199 * v[39] + -0.0113645261153579 * v[35] + -0.0146686462685466 * v[36] + v[37];
   y[126] = 0.000816664425656199 * v[29] + -0.0113645261153579 * v[31] + -0.0146686462685466 * v[30] + v[28];
   y[128] = 0.00100000004749745 * v[27] + 0.00100000004749745 * v[45] + -0.028999999165535 * v[46] + y[132];
   y[129] = 0.00100000004749745 * v[47] + 0.00100000004749745 * v[48] + -0.028999999165535 * v[49] + y[133];
   y[130] = 0.00100000004749745 * v[20] + 0.00100000004749745 * v[44] + -0.028999999165535 * v[43] + y[134];
   y[135] = v[27] + 2.22044604925031e-16 * v[45] + 2.22044604925031e-16 * v[46];
   y[136] = v[47] + 2.22044604925031e-16 * v[48] + 2.22044604925031e-16 * v[49];
   y[137] = v[20] + 2.22044604925031e-16 * v[44] + 2.22044604925031e-16 * v[43];
   y[138] = -2.22044604925031e-16 * v[27] + v[45];
   y[139] = -2.22044604925031e-16 * v[47] + v[48];
   y[140] = -2.22044604925031e-16 * v[20] + v[44];
   y[141] = -2.22044604925031e-16 * v[27] + -4.93038065763132e-32 * v[45] + v[46];
   y[142] = -2.22044604925031e-16 * v[47] + -4.93038065763132e-32 * v[48] + v[49];
   y[143] = -2.22044604925031e-16 * v[20] + -4.93038065763132e-32 * v[44] + v[43];
   // dependent variables without operations
   y[0] = 0.;
   y[1] = 0.;
   y[2] = 0.1625;
   y[3] = 0.100000001490116;
   y[7] = 0.0780000016093254;
   y[11] = 0.0769999995827675;
   y[15] = 0.061999998986721;
   y[19] = 0.0610000006854534;
   y[23] = 0.0769999995827675;
   y[27] = 0.0610000006854534;
   y[31] = 0.0700000002980232;
   y[35] = 0.0799999982118607;
   y[39] = 0.0719999969005585;
   y[43] = 0.0570000000298023;
   y[47] = 0.0570000000298023;
   y[51] = 0.0570000000298023;
   y[55] = 0.0560000017285347;
   y[59] = 0.0649999976158142;
   y[63] = 0.0570000000298023;
   y[67] = 0.0520000010728836;
   y[71] = 0.0570000000298023;
   y[75] = 0.0469999983906746;
   y[79] = 0.0469999983906746;
   y[83] = 0.0450000017881393;
   y[87] = 0.046000000089407;
   y[91] = 0.0469999983906746;
   y[95] = 0.0469999983906746;
   y[99] = 0.0469999983906746;
   y[103] = 0.0469999983906746;
   y[107] = 0.0430000014603138;
   y[108] = 0.;
   y[109] = 0.;
   y[110] = 0.1625;
   y[111] = 0.100000001490116;
   y[115] = 0.286502420902252;
   y[119] = 0.257050633430481;
   y[123] = 0.0756581649184227;
   y[127] = 0.0707167387008667;
   y[131] = 0.0430000014603138;

        





//
// environment vs. robot collisions
//

// wrist_3_link
if (sphere_environment_in_collision(environment,
                                    y[128],
                                    y[129],
                                    y[130],
                                    y[131])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[104],
                                        y[105],
                                        y[106],
                                        y[107])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// wrist_2_link
if (sphere_environment_in_collision(environment,
                                    y[124],
                                    y[125],
                                    y[126],
                                    y[127])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[88],
                                        y[89],
                                        y[90],
                                        y[91])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[92],
                                        y[93],
                                        y[94],
                                        y[95])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[96],
                                        y[97],
                                        y[98],
                                        y[99])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[100],
                                        y[101],
                                        y[102],
                                        y[103])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// wrist_1_link
if (sphere_environment_in_collision(environment,
                                    y[120],
                                    y[121],
                                    y[122],
                                    y[123])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[72],
                                        y[73],
                                        y[74],
                                        y[75])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[76],
                                        y[77],
                                        y[78],
                                        y[79])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[80],
                                        y[81],
                                        y[82],
                                        y[83])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[84],
                                        y[85],
                                        y[86],
                                        y[87])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// forearm_link
if (sphere_environment_in_collision(environment,
                                    y[116],
                                    y[117],
                                    y[118],
                                    y[119])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[36],
                                        y[37],
                                        y[38],
                                        y[39])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[40],
                                        y[41],
                                        y[42],
                                        y[43])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[44],
                                        y[45],
                                        y[46],
                                        y[47])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[48],
                                        y[49],
                                        y[50],
                                        y[51])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[52],
                                        y[53],
                                        y[54],
                                        y[55])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[56],
                                        y[57],
                                        y[58],
                                        y[59])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[60],
                                        y[61],
                                        y[62],
                                        y[63])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[64],
                                        y[65],
                                        y[66],
                                        y[67])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[68],
                                        y[69],
                                        y[70],
                                        y[71])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// upper_arm_link
if (sphere_environment_in_collision(environment,
                                    y[112],
                                    y[113],
                                    y[114],
                                    y[115])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[4],
                                        y[5],
                                        y[6],
                                        y[7])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[8],
                                        y[9],
                                        y[10],
                                        y[11])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[12],
                                        y[13],
                                        y[14],
                                        y[15])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[16],
                                        y[17],
                                        y[18],
                                        y[19])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[20],
                                        y[21],
                                        y[22],
                                        y[23])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[24],
                                        y[25],
                                        y[26],
                                        y[27])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[28],
                                        y[29],
                                        y[30],
                                        y[31])) [[unlikely]]
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[32],
                                        y[33],
                                        y[34],
                                        y[35])) [[unlikely]]
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// shoulder_link
if (sphere_environment_in_collision(environment,
                                    y[108],
                                    y[109],
                                    y[110],
                                    y[111])) [[unlikely]]
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3])) [[unlikely]]
    {
        return false;
    }
    
}



//
// robot self-collisions
//












// shoulder_link vs. forearm_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[108],
                                                 y[109],
                                                 y[110],
                                                 y[111],
                                                 y[116],
                                                 y[117],
                                                 y[118],
                                                 y[119])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[36 ],
                                                     y[37 ],
                                                     y[38 ],
                                                     y[39 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[40 ],
                                                     y[41 ],
                                                     y[42 ],
                                                     y[43 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[44 ],
                                                     y[45 ],
                                                     y[46 ],
                                                     y[47 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[48 ],
                                                     y[49 ],
                                                     y[50 ],
                                                     y[51 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[52 ],
                                                     y[53 ],
                                                     y[54 ],
                                                     y[55 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[56 ],
                                                     y[57 ],
                                                     y[58 ],
                                                     y[59 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[60 ],
                                                     y[61 ],
                                                     y[62 ],
                                                     y[63 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[64 ],
                                                     y[65 ],
                                                     y[66 ],
                                                     y[67 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[68 ],
                                                     y[69 ],
                                                     y[70 ],
                                                     y[71 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// shoulder_link vs. wrist_1_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[108],
                                                 y[109],
                                                 y[110],
                                                 y[111],
                                                 y[120],
                                                 y[121],
                                                 y[122],
                                                 y[123])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// shoulder_link vs. wrist_2_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[108],
                                                 y[109],
                                                 y[110],
                                                 y[111],
                                                 y[124],
                                                 y[125],
                                                 y[126],
                                                 y[127])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// shoulder_link vs. wrist_3_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[108],
                                                 y[109],
                                                 y[110],
                                                 y[111],
                                                 y[128],
                                                 y[129],
                                                 y[130],
                                                 y[131])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[0 ],
                                                     y[1 ],
                                                     y[2 ],
                                                     y[3 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// upper_arm_link vs. wrist_1_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[112],
                                                 y[113],
                                                 y[114],
                                                 y[115],
                                                 y[120],
                                                 y[121],
                                                 y[122],
                                                 y[123])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[72 ],
                                                     y[73 ],
                                                     y[74 ],
                                                     y[75 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[76 ],
                                                     y[77 ],
                                                     y[78 ],
                                                     y[79 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[80 ],
                                                     y[81 ],
                                                     y[82 ],
                                                     y[83 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[84 ],
                                                     y[85 ],
                                                     y[86 ],
                                                     y[87 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// upper_arm_link vs. wrist_2_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[112],
                                                 y[113],
                                                 y[114],
                                                 y[115],
                                                 y[124],
                                                 y[125],
                                                 y[126],
                                                 y[127])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[88 ],
                                                     y[89 ],
                                                     y[90 ],
                                                     y[91 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[92 ],
                                                     y[93 ],
                                                     y[94 ],
                                                     y[95 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[96 ],
                                                     y[97 ],
                                                     y[98 ],
                                                     y[99 ])) [[unlikely]]
    {
        return false;
    }

    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[100 ],
                                                     y[101 ],
                                                     y[102 ],
                                                     y[103 ])) [[unlikely]]
    {
        return false;
    }

    
    
}











// upper_arm_link vs. wrist_3_link
if (sphere_sphere_self_collision<decltype(x[0])>(y[112],
                                                 y[113],
                                                 y[114],
                                                 y[115],
                                                 y[128],
                                                 y[129],
                                                 y[130],
                                                 y[131])) [[unlikely]]
{
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[4 ],
                                                     y[5 ],
                                                     y[6 ],
                                                     y[7 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[8 ],
                                                     y[9 ],
                                                     y[10 ],
                                                     y[11 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[12 ],
                                                     y[13 ],
                                                     y[14 ],
                                                     y[15 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[16 ],
                                                     y[17 ],
                                                     y[18 ],
                                                     y[19 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[20 ],
                                                     y[21 ],
                                                     y[22 ],
                                                     y[23 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[24 ],
                                                     y[25 ],
                                                     y[26 ],
                                                     y[27 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[28 ],
                                                     y[29 ],
                                                     y[30 ],
                                                     y[31 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
    

    
    

    if (sphere_sphere_self_collision<decltype(x[0])>(y[32 ],
                                                     y[33 ],
                                                     y[34 ],
                                                     y[35 ],
                                                     y[104 ],
                                                     y[105 ],
                                                     y[106 ],
                                                     y[107 ])) [[unlikely]]
    {
        return false;
    }

    
    
}




        // attaching at tool0
        set_attachment_pose(environment, to_isometry(&y[132]));

        //
        // attachment vs. environment collisions
        //
        if (attachment_environment_collision(environment)) [[unlikely]]
        {
            return false;
        }

        //
        // attachment vs. robot collisions
        //

        
        
        
        

        // Attachment vs. shoulder_link
        if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                        y[108],
                                                        y[109],
                                                        y[110],
                                                        y[111])) [[unlikely]]
        {
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[0],
                                                            y[1],
                                                            y[2],
                                                            y[3])) [[unlikely]]
            {
                return false;
            }
            
        }
        
        
        
        

        // Attachment vs. upper_arm_link
        if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                        y[112],
                                                        y[113],
                                                        y[114],
                                                        y[115])) [[unlikely]]
        {
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[4],
                                                            y[5],
                                                            y[6],
                                                            y[7])) [[unlikely]]
            {
                return false;
            }
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[8],
                                                            y[9],
                                                            y[10],
                                                            y[11])) [[unlikely]]
            {
                return false;
            }
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[12],
                                                            y[13],
                                                            y[14],
                                                            y[15])) [[unlikely]]
            {
                return false;
            }
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[16],
                                                            y[17],
                                                            y[18],
                                                            y[19])) [[unlikely]]
            {
                return false;
            }
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[20],
                                                            y[21],
                                                            y[22],
                                                            y[23])) [[unlikely]]
            {
                return false;
            }
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[24],
                                                            y[25],
                                                            y[26],
                                                            y[27])) [[unlikely]]
            {
                return false;
            }
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[28],
                                                            y[29],
                                                            y[30],
                                                            y[31])) [[unlikely]]
            {
                return false;
            }
            
            
            if (attachment_sphere_collision<decltype(x[0])>(environment,
                                                            y[32],
                                                            y[33],
                                                            y[34],
                                                            y[35])) [[unlikely]]
            {
                return false;
            }
            
        }
        

        return true;
    }

    static inline auto eefk(const std::array<float, 6> &x) noexcept -> Eigen::Isometry3f
    {
        std::array<float, 26> v;
        std::array<float, 12> y;

           v[0] = cos(x[0]);
   v[1] = sin(x[0]);
   v[2] = -1. * v[0] + -1.22464679914735e-16 * v[1];
   v[3] = cos(x[1]);
   v[1] = 1.22464679914735e-16 * v[0] + -1. * v[1];
   v[0] = 0. - v[1];
   v[4] = sin(x[1]);
   v[5] = -2.05103489747671e-10 * v[4];
   v[6] = v[2] * v[3] + v[0] * v[5];
   v[7] = cos(x[2]);
   v[8] = -1. * v[4] - -2.05103489747671e-10 * v[5];
   v[9] = -2.05103489747671e-10 * v[3];
   v[10] = v[2] * v[8] + v[0] * v[9];
   v[11] = sin(x[2]);
   v[12] = v[6] * v[7] + v[10] * v[11];
   v[0] = -1. * v[0];
   v[13] = sin(x[3]);
   v[14] = 0. - v[13];
   v[15] = 0. - v[11];
   v[10] = v[6] * v[15] + v[10] * v[7];
   v[16] = cos(x[3]);
   v[17] = v[12] * v[14] + v[10] * v[16];
   v[10] = v[12] * v[16] + v[10] * v[13];
   v[18] = sin(x[4]);
   v[19] = -2.05103489747671e-10 * v[18];
   v[20] = -1. * v[18] - -2.05103489747671e-10 * v[19];
   v[21] = cos(x[4]);
   v[22] = -2.05103489747671e-10 * v[21];
   v[23] = 0. - -1. * v[21];
   v[24] = v[10] * v[20] + v[17] * v[22] + v[0] * v[23];
   v[25] = -1. * v[17] + -2.05103489747671e-10 * v[0];
   y[0] = -0.425 * v[6] + -0.3922 * v[12] + 0.1333 * v[0] + -0.0997 * v[17] + -2.04488118229785e-11 * v[0] + 0.0996 * v[24] + -2.0428301480127e-11 * v[25];
   v[5] = v[1] * v[3] + v[2] * v[5];
   v[9] = v[1] * v[8] + v[2] * v[9];
   v[8] = v[5] * v[7] + v[9] * v[11];
   v[2] = -1. * v[2];
   v[9] = v[5] * v[15] + v[9] * v[7];
   v[1] = v[8] * v[14] + v[9] * v[16];
   v[9] = v[8] * v[16] + v[9] * v[13];
   v[12] = v[9] * v[20] + v[1] * v[22] + v[2] * v[23];
   v[6] = -1. * v[1] + -2.05103489747671e-10 * v[2];
   y[1] = -0.425 * v[5] + -0.3922 * v[8] + 0.1333 * v[2] + -0.0997 * v[1] + -2.04488118229785e-11 * v[2] + 0.0996 * v[12] + -2.0428301480127e-11 * v[6];
   v[3] = 0. - -1. * v[3];
   v[11] = v[4] * v[7] + v[3] * v[11];
   v[3] = v[4] * v[15] + v[3] * v[7];
   v[14] = v[11] * v[14] + v[3] * v[16];
   v[3] = v[11] * v[16] + v[3] * v[13];
   v[23] = v[3] * v[20] + v[14] * v[22] + -2.05103489747671e-10 * v[23];
   v[22] = 4.2067441506673e-20 + -1. * v[14];
   y[2] = 0.1625 + -0.425 * v[4] + -2.73402951833646e-11 + -0.3922 * v[11] + 4.19412266608633e-21 + -0.0997 * v[14] + 0.0996 * v[23] + -2.0428301480127e-11 * v[22];
   v[10] = v[10] * v[21] + v[17] * v[19] + v[0] * v[18];
   v[17] = cos(x[5]);
   v[0] = sin(x[5]);
   v[11] = v[17] + -1.22464679939853e-16 * v[0];
   v[4] = -1.22464679914735e-16 * v[17] + -2.05103489747671e-10 * v[0];
   v[0] = -1.22464679914735e-16 * v[17] + -1. * v[0];
   v[17] = v[10] * v[11] + v[24] * v[4] + v[25] * v[0];
   v[20] = v[0] - -2.05103489747671e-10 * v[4];
   v[16] = -2.05103489747671e-10 * v[11] - 1.22464679889617e-16 * v[0];
   v[13] = 1.22464679889617e-16 * v[4] - v[11];
   v[15] = v[10] * v[20] + v[24] * v[16] + v[25] * v[13];
   v[10] = 1.22464679889617e-16 * v[10] + v[24] + -2.05103489747671e-10 * v[25];
   y[3] = v[17] + 2.22044604925031e-16 * v[15] + 2.22044604925031e-16 * v[10];
   v[9] = v[9] * v[21] + v[1] * v[19] + v[2] * v[18];
   v[1] = v[9] * v[11] + v[12] * v[4] + v[6] * v[0];
   v[2] = v[9] * v[20] + v[12] * v[16] + v[6] * v[13];
   v[9] = 1.22464679889617e-16 * v[9] + v[12] + -2.05103489747671e-10 * v[6];
   y[4] = v[1] + 2.22044604925031e-16 * v[2] + 2.22044604925031e-16 * v[9];
   v[3] = v[3] * v[21] + v[14] * v[19] + -2.05103489747671e-10 * v[18];
   v[0] = v[3] * v[11] + v[23] * v[4] + v[22] * v[0];
   v[13] = v[3] * v[20] + v[23] * v[16] + v[22] * v[13];
   v[3] = 1.22464679889617e-16 * v[3] + v[23] + -2.05103489747671e-10 * v[22];
   y[5] = v[0] + 2.22044604925031e-16 * v[13] + 2.22044604925031e-16 * v[3];
   y[6] = -2.22044604925031e-16 * v[17] + v[15];
   y[7] = -2.22044604925031e-16 * v[1] + v[2];
   y[8] = -2.22044604925031e-16 * v[0] + v[13];
   y[9] = -2.22044604925031e-16 * v[17] + -4.93038065763132e-32 * v[15] + v[10];
   y[10] = -2.22044604925031e-16 * v[1] + -4.93038065763132e-32 * v[2] + v[9];
   y[11] = -2.22044604925031e-16 * v[0] + -4.93038065763132e-32 * v[13] + v[3];


        return to_isometry(y.data());
    }
};
}

// NOLINTEND(*-magic-numbers)
