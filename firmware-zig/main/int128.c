// The 128-bit integer helpers the Zig C backend calls for u128/i128
// multiplication, division and remainder (zig.h's zig_mul_i128,
// zig_divTrunc_u128 and so on). GCC provides them as __multi3 and friends
// only on 64-bit targets, so the 32-bit Xtensa libgcc lacks them. Zig's own
// float formatting needs them, which is how they end up in ping.c.
//
// Without __int128, zig.h passes 128-bit integers as this struct, low half
// first on a little-endian target, so these take and return the same.
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint64_t lo;
    uint64_t hi;
} u128;

typedef struct {
    uint64_t lo;
    int64_t hi;
} i128;

u128 __multi3(u128 a, u128 b);
u128 __udivti3(u128 a, u128 b);
u128 __umodti3(u128 a, u128 b);
i128 __divti3(i128 a, i128 b);
i128 __modti3(i128 a, i128 b);

// The full 128-bit product of two 64-bit numbers, from 32-bit halves.
static u128 mul_u64(uint64_t a, uint64_t b)
{
    const uint64_t a_lo = (uint32_t)a, a_hi = a >> 32;
    const uint64_t b_lo = (uint32_t)b, b_hi = b >> 32;
    const uint64_t ll = a_lo * b_lo;
    const uint64_t lh = a_lo * b_hi;
    const uint64_t hl = a_hi * b_lo;
    const uint64_t hh = a_hi * b_hi;
    const uint64_t mid = (ll >> 32) + (uint32_t)lh + (uint32_t)hl;
    return (u128){
        .lo = (mid << 32) | (uint32_t)ll,
        .hi = hh + (lh >> 32) + (hl >> 32) + (mid >> 32),
    };
}

// The low 128 bits of the product, which are the same signed or unsigned.
u128 __multi3(u128 a, u128 b)
{
    u128 r = mul_u64(a.lo, b.lo);
    r.hi += a.lo * b.hi + a.hi * b.lo;
    return r;
}

static bool lt(u128 a, u128 b)
{
    return a.hi < b.hi || (a.hi == b.hi && a.lo < b.lo);
}

static u128 sub(u128 a, u128 b)
{
    return (u128){ .lo = a.lo - b.lo, .hi = a.hi - b.hi - (a.lo < b.lo) };
}

static u128 neg(u128 a)
{
    return sub((u128){ 0, 0 }, a);
}

// Restoring division, one bit at a time. Slow, but only formatting uses it.
// Division by zero is caught by the Zig code's safety checks first.
static u128 divmod(u128 n, u128 d, u128 *rem)
{
    u128 q = { 0, 0 }, r = { 0, 0 };
    for (int i = 127; i >= 0; i--) {
        r.hi = (r.hi << 1) | (r.lo >> 63);
        r.lo = (r.lo << 1) | ((i >= 64 ? n.hi >> (i - 64) : n.lo >> i) & 1);
        if (!lt(r, d)) {
            r = sub(r, d);
            if (i >= 64) {
                q.hi |= (uint64_t)1 << (i - 64);
            } else {
                q.lo |= (uint64_t)1 << i;
            }
        }
    }
    *rem = r;
    return q;
}

u128 __udivti3(u128 a, u128 b)
{
    u128 r;
    return divmod(a, b, &r);
}

u128 __umodti3(u128 a, u128 b)
{
    u128 r;
    divmod(a, b, &r);
    return r;
}

static u128 abs_i128(i128 a)
{
    const u128 u = { a.lo, (uint64_t)a.hi };
    return a.hi < 0 ? neg(u) : u;
}

static i128 to_i128(u128 u, bool negative)
{
    if (negative) {
        u = neg(u);
    }
    return (i128){ u.lo, (int64_t)u.hi };
}

// Truncating, as C and Zig's @divTrunc do: the quotient rounds toward zero
// and the remainder takes the dividend's sign.
i128 __divti3(i128 a, i128 b)
{
    u128 r;
    return to_i128(divmod(abs_i128(a), abs_i128(b), &r), (a.hi < 0) != (b.hi < 0));
}

i128 __modti3(i128 a, i128 b)
{
    u128 r;
    divmod(abs_i128(a), abs_i128(b), &r);
    return to_i128(r, a.hi < 0);
}
