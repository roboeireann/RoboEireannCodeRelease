/**
 * @file: MacroUtils.h
 * 
 * helper code for macros - centralized here to avoid unnecessary duplication
 * 
 * @author: Rudi Villing
 */

#pragma once


/** 
 * force a variable that is used in some macro expansions but not in others
 * to always be used to prevent compiler warnings
 */
#define HBS__UNUSED(_var) static_cast<void>(_var)

/**
 * Convert arbitrary code to a statement that can be terminated with a semicolon
 */
#define HBS__STATEMENT(...) do { __VA_ARGS__; } while(0)


/**
 * Concatenate exactly two parameters
 */
#define HBS__JOIN(a_, b_) HBS__JOIN_I(a_, b_)
#define HBS__JOIN_I(a_, b_) a_##b_

/** 
 * Concatenate the first two parameters - multiple levels of expansion to ensure parameters can be joined. 
 * */
// #define HBS__JOIN(_a, ...) HBS__JOIN_I(_a, __VA_ARGS__)
// #define HBS__JOIN_I(_a, ...) HBS__JOIN_II(_a ## __VA_ARGS__)
// #define HBS__JOIN_II(...) __VA_ARGS__

/**
 * evaluate a parameter ensuring that any macro substitution has occurred.
 * Overcomes some odd parameter issues that would otherwise require nesting multiple
 * macro levels
 */
#define HBS__EVAL(...) __VA_ARGS__

/**
 * Determine the number of args to a macro (only designed for small numbers of arguments).
 * 
 * Example:
 *       _NARGS(a, b, c)
 * -->   _NARGS_I(a,b,c, 5,4,3,2,1,0)
 * -->   _NARGS_II(a,b,c, 5,4,3,2,1,0)
 * -->   3
 */
#define HBS__NARGS(...)       HBS__NARGS_I(__VA_ARGS__, HBS__NARGS_III)
#define HBS__NARGS_I(...)     HBS__NARGS_II(__VA_ARGS__)
#define HBS__NARGS_II(_0, _1, _2, _3, _4, N, ...)  N
#define HBS__NARGS_III        5, 4, 3, 2, 1, 0

/**
 * support overloading of macros with different numbers of parameters
 * 
 * Automatically redirect to the appropriate overloaded macro based
 * on the base macro name (prefix) and arg list.
 * 
 * The overloaded macros are given by the prefix followed by _N where N is
 * the number of params (which must be 1 or larger)
 * 
 * WARNING --- does not work for zero params!
 * 
 * Declare the actual overrides with the appropriate number of params to match their name
 * 
 * Example: 
 * 
 *    #define SOME_MACRO(...) HBS__OVERLOAD(_SOME_MACRO, __VA_ARGS__)
 * 
 *    // add support for 1, or 2 argument versions of SOME_MACRO 
 *    #define _SOME_MACRO_1(a)   fillInTheCodeHere...
 *    #define _SOME_MACRO_2(a,b) fillInTheCodeHere...
 * 
 * @param _name the base name/prefix to be used for the macros to be selected
 * 
 * see: https://stackoverflow.com/questions/11761703/overloading-macro-on-number-of-arguments
 */
#define HBS__OVERLOAD(_name, ...)   HBS__OVERLOAD_I(_name, HBS__NARGS(__VA_ARGS__))(__VA_ARGS__)
#define HBS__OVERLOAD_I(_name, _n)  HBS__OVERLOAD_II(_name, _n)
#define HBS__OVERLOAD_II(_name, _n) _name##_##_n

/**
 * FIXME - delete
 * support for zero or just a small few args
 */
// #define _ARG5(_0, _1, _2, _3, _4, _5, ...) _5
// #define _HAS_COMMA(...) _ARG5(__VA_ARGS__, 1, 1, 1, 1, 0)
// #define _TRIGGER_PARENTHESIS_(...) ,

/**
 * support for a single optional parameter
 * 
 * Example: 
 * 
 *    #define SOME_MACRO(...) HBS__OPTIONAL(_SOME_MACRO, __VA_ARGS__)
 * 
 *    // add support for 0, or 1 argument versions depending on whether option provided or not
 *    #define _SOME_MACRO_0()  fillInTheCodeHere...
 *    #define _SOME_MACRO_1(a) fillInTheCodeHere...
*/
#define HBS__SEL_2(_0, _1, _2, ...) _2
#define HBS__COMMA_FUNC(...) ,
#define HBS__COMMA_IF_VALID(...) HBS__COMMA_FUNC __VA_ARGS__ (/*empty*/)
#define HBS__HAS_OPTION(...) HBS__HAS_OPTION_I(HBS__COMMA_IF_VALID(__VA_ARGS__))
#define HBS__HAS_OPTION_I(...) HBS__SEL_2(__VA_ARGS__, 0, 1)

#define HBS__OPTIONAL(_name, ...) HBS__OVERLOAD_I(_name, HBS__HAS_OPTION(__VA_ARGS__))(__VA_ARGS__)


/**
 * Remove optional parentheses
 * 
 * see https://www.mikeash.com/pyblog/friday-qa-2015-03-20-preprocessor-abuse-and-optional-parentheses.html
 */


// remove one level of parentheses that may or may not be present
#define HBS__STRIP_OPT_PAREN(...) _JOIN(HBS__DO_NOT, HBS__STRIP_OPT_PAREN_I __VA_ARGS__)
#define HBS__STRIP_OPT_PAREN_I(...) HBS__STRIP_OPT_PAREN_I __VA_ARGS__
#define HBS__DO_NOT_STRIP_OPT_PAREN_I

// remove one level of parentheses that must be present
#define HBS__STRIP_REQ_PAREN(...) HBS__STRIP_REQ_PAREN_I __VA_ARGS__
#define HBS__STRIP_REQ_PAREN_I(...) __VA_ARGS__



// --- List Mapping Macros (Max 30) ---
// These macros perform the "looping" logic.
// They count the number of arguments provided and select the appropriate 
// HBS__MAP_N macro to expand them.
//
// HBS__GET_MAP selects the Nth argument, which corresponds to the correct macro name.
// We expand to 30 to support large argument lists or state variables.

#define HBS__EXPAND(x) x

// Dispatcher: Selects the correct HBS__MAP_N macro based on argument count.
#define HBS__GET_MAP( \
    _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, \
    _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, \
    _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, \
    NAME, ...) NAME

// intentionally empty definition
#define HBS__MAP_SENTINEL

#if 0
// Main Map Macro: Calls the dispatcher.
#define HBS__MAP(M, D, ...) HBS__EXPAND(HBS__GET_MAP(HBS__MAP_SENTINEL __VA_OPT__(,) __VA_ARGS__, \
    HBS__MAP_30, HBS__MAP_29, HBS__MAP_28, HBS__MAP_27, HBS__MAP_26, \
    HBS__MAP_25, HBS__MAP_24, HBS__MAP_23, HBS__MAP_22, HBS__MAP_21, \
    HBS__MAP_20, HBS__MAP_19, HBS__MAP_18, HBS__MAP_17, HBS__MAP_16, \
    HBS__MAP_15, HBS__MAP_14, HBS__MAP_13, HBS__MAP_12, HBS__MAP_11, \
    HBS__MAP_10, HBS__MAP_9,  HBS__MAP_8,  HBS__MAP_7,  HBS__MAP_6, \
    HBS__MAP_5,  HBS__MAP_4,  HBS__MAP_3,  HBS__MAP_2,  HBS__MAP_1, \
    HBS__MAP_0)(M, D __VA_OPT__(,) __VA_ARGS__))

// Implementation of the map for N items.
// M is the macro to apply to the item.
// D is the delimiter to put between items.
// "M _1" is a juxtaposition: when _1 is a parenthesised tuple like (a,b,c),
// the preprocessor resolves it as a function-like call M(a,b,c).
// When _1 is a bare token like "taskA", M must be object-like or the call
// must be made explicitly — see HBS__MAPC (call-style) variants below.
#define HBS__MAP_0(M, D)
#define HBS__MAP_1(M, D, _1) M _1
#define HBS__MAP_2(M, D, _1, _2) M _1 D M _2
#define HBS__MAP_3(M, D, _1, _2, _3) M _1 D M _2 D M _3
#define HBS__MAP_4(M, D, _1, _2, _3, _4) M _1 D M _2 D M _3 D M _4
#define HBS__MAP_5(M, D, _1, _2, _3, _4, _5) M _1 D M _2 D M _3 D M _4 D M _5
#define HBS__MAP_6(M, D, _1, _2, _3, _4, _5, _6) M _1 D M _2 D M _3 D M _4 D M _5 D M _6
#define HBS__MAP_7(M, D, _1, _2, _3, _4, _5, _6, _7) M _1 D M _2 D M _3 D M _4 D M _5 D M _6 D M _7
#define HBS__MAP_8(M, D, _1, _2, _3, _4, _5, _6, _7, _8) M _1 D M _2 D M _3 D M _4 D M _5 D M _6 D M _7 D M _8
#define HBS__MAP_9(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9) \
    M _1 D M _2 D M _3 D M _4 D M _5 D M _6 D M _7 D M _8 D M _9
#define HBS__MAP_10(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10) \
    M _1 D M _2 D M _3 D M _4 D M _5 D M _6 D M _7 D M _8 D M _9 D M _10
#define HBS__MAP_11(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11) \
    HBS__MAP_10(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10) D M _11
#define HBS__MAP_12(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12) \
    HBS__MAP_11(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11) D M _12
#define HBS__MAP_13(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13) \
    HBS__MAP_12(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12) D M _13
#define HBS__MAP_14(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14) \
    HBS__MAP_13(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13) D M _14
#define HBS__MAP_15(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15) \
    HBS__MAP_14(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14) D M _15
#define HBS__MAP_16(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16) \
    HBS__MAP_15(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15) D M _16
#define HBS__MAP_17(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17) \
    HBS__MAP_16(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16) D M _17
#define HBS__MAP_18(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18) \
    HBS__MAP_17(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17) D M _18
#define HBS__MAP_19(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19) \
    HBS__MAP_18(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18) D M _19
#define HBS__MAP_20(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20) \
    HBS__MAP_19(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19) D M _20
#define HBS__MAP_21(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21) \
    HBS__MAP_20(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20) D M _21
#define HBS__MAP_22(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22) \
    HBS__MAP_21(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21) D M _22
#define HBS__MAP_23(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23) \
    HBS__MAP_22(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22) D M _23
#define HBS__MAP_24(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24) \
    HBS__MAP_23(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23) D M _24
#define HBS__MAP_25(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25) \
    HBS__MAP_24(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24) D M _25
#define HBS__MAP_26(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26) \
    HBS__MAP_25(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25) D M _26
#define HBS__MAP_27(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27) \
    HBS__MAP_26(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26) D M _27
#define HBS__MAP_28(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28) \
    HBS__MAP_27(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27) D M _28
#define HBS__MAP_29(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29) \
    HBS__MAP_28(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28) D M _29
#define HBS__MAP_30(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30) \
    HBS__MAP_29(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29) D M _30
#endif

// ===========================================================================
// HBS__MAPC — "call-style" map for bare-name items
// ===========================================================================
//
// HBS__MAP uses "M _1" juxtaposition which works when _1 is a parenthesised
// tuple (the preprocessor resolves it as M(contents...)).  For bare tokens
// like method names in HBS_SEQUENCE/HBS_FALLBACK, we need explicit M(_1).
// HBS__MAPC provides this: it wraps each item in parens before calling M.
// Only the small sizes needed by HBS_SEQUENCE/HBS_FALLBACK are defined here.

#define HBS__MAPC(M, D, ...) HBS__EXPAND(HBS__GET_MAP(HBS__MAP_SENTINEL __VA_OPT__(,) __VA_ARGS__, \
    HBS__MAPC_30, HBS__MAPC_29, HBS__MAPC_28, HBS__MAPC_27, HBS__MAPC_26, \
    HBS__MAPC_25, HBS__MAPC_24, HBS__MAPC_23, HBS__MAPC_22, HBS__MAPC_21, \
    HBS__MAPC_20, HBS__MAPC_19, HBS__MAPC_18, HBS__MAPC_17, HBS__MAPC_16, \
    HBS__MAPC_15, HBS__MAPC_14, HBS__MAPC_13, HBS__MAPC_12, HBS__MAPC_11, \
    HBS__MAPC_10, HBS__MAPC_9,  HBS__MAPC_8,  HBS__MAPC_7,  HBS__MAPC_6, \
    HBS__MAPC_5,  HBS__MAPC_4,  HBS__MAPC_3,  HBS__MAPC_2,  HBS__MAPC_1, \
    HBS__MAPC_0)(M, D __VA_OPT__(,) __VA_ARGS__))

#define HBS__MAPC_0(M, D)
#define HBS__MAPC_1(M, D, _1) M(_1)
#define HBS__MAPC_2(M, D, _1, _2) M(_1) D M(_2)
#define HBS__MAPC_3(M, D, _1, _2, _3) M(_1) D M(_2) D M(_3)
#define HBS__MAPC_4(M, D, _1, _2, _3, _4) M(_1) D M(_2) D M(_3) D M(_4)
#define HBS__MAPC_5(M, D, _1, _2, _3, _4, _5) M(_1) D M(_2) D M(_3) D M(_4) D M(_5)
#define HBS__MAPC_6(M, D, _1, _2, _3, _4, _5, _6) M(_1) D M(_2) D M(_3) D M(_4) D M(_5) D M(_6)
#define HBS__MAPC_7(M, D, _1, _2, _3, _4, _5, _6, _7) M(_1) D M(_2) D M(_3) D M(_4) D M(_5) D M(_6) D M(_7)
#define HBS__MAPC_8(M, D, _1, _2, _3, _4, _5, _6, _7, _8) M(_1) D M(_2) D M(_3) D M(_4) D M(_5) D M(_6) D M(_7) D M(_8)
#define HBS__MAPC_9(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9) \
    M(_1) D M(_2) D M(_3) D M(_4) D M(_5) D M(_6) D M(_7) D M(_8) D M(_9)
#define HBS__MAPC_10(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10) \
    M(_1) D M(_2) D M(_3) D M(_4) D M(_5) D M(_6) D M(_7) D M(_8) D M(_9) D M(_10)
#define HBS__MAPC_11(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11) \
    HBS__MAPC_10(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10) D M(_11)
#define HBS__MAPC_12(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12) \
    HBS__MAPC_11(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11) D M(_12)
#define HBS__MAPC_13(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13) \
    HBS__MAPC_12(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12) D M(_13)
#define HBS__MAPC_14(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14) \
    HBS__MAPC_13(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13) D M(_14)
#define HBS__MAPC_15(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15) \
    HBS__MAPC_14(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14) D M(_15)
#define HBS__MAPC_16(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16) \
    HBS__MAPC_15(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15) D M(_16)
#define HBS__MAPC_17(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17) \
    HBS__MAPC_16(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16) D M(_17)
#define HBS__MAPC_18(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18) \
    HBS__MAPC_17(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17) D M(_18)
#define HBS__MAPC_19(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19) \
    HBS__MAPC_18(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18) D M(_19)
#define HBS__MAPC_20(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20) \
    HBS__MAPC_19(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19) D M(_20)
#define HBS__MAPC_21(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21) \
    HBS__MAPC_20(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20) D M(_21)
#define HBS__MAPC_22(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22) \
    HBS__MAPC_21(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21) D M(_22)
#define HBS__MAPC_23(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23) \
    HBS__MAPC_22(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22) D M(_23)
#define HBS__MAPC_24(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24) \
    HBS__MAPC_23(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23) D M(_24)
#define HBS__MAPC_25(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25) \
    HBS__MAPC_24(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24) D M(_25)
#define HBS__MAPC_26(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26) \
    HBS__MAPC_25(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25) D M(_26)
#define HBS__MAPC_27(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27) \
    HBS__MAPC_26(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26) D M(_27)
#define HBS__MAPC_28(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28) \
    HBS__MAPC_27(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27) D M(_28)
#define HBS__MAPC_29(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29) \
    HBS__MAPC_28(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28) D M(_29)
#define HBS__MAPC_30(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30) \
    HBS__MAPC_29(M, D, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29) D M(_30)


// ===========================================================================
// Tuple-element field/ref/arg helpers (used by CORO_DEFINE / HSM_DEFINE)
// ===========================================================================
//
// Each ARGS/VARS/DEFINES_PARAMS element is a parenthesised tuple: (Type, Name)
// or (Type, Name, Default).
//
// HBS__MAPC passes each item as a single parenthesised argument M(ITEM), so
// the helpers receive the whole tuple as one arg.  A one-level indirection
// strips the parens: HBS__ARG_FIELD((float, speed, 1.0f)) expands to
// HBS__ARG_FIELD_I(float, speed, 1.0f), which receives T/N/default separately.

// --- ARGS element helpers ---
// Emit a struct field:  "Type Name {Default};"  or  "Type Name;"
#define HBS__ARG_FIELD(tuple_)         HBS__ARG_FIELD_I tuple_
#define HBS__ARG_FIELD_I(T, N, ...)    T N __VA_OPT__(= __VA_ARGS__);

// Emit ", varsPtr->Name"  (actual argument when calling _bodyCode)
#define HBS__VAR_REF(tuple_)           HBS__VAR_REF_I tuple_
#define HBS__VAR_REF_I(T, N, ...)      , varsPtr->N

// Emit ", [[maybe_unused]] Type& Name"  (parameter in _bodyCode signature)
#define HBS__VAR_ARG(tuple_)           HBS__VAR_ARG_I tuple_
#define HBS__VAR_ARG_I(T, N, ...)      , [[maybe_unused]] T& N

// --- VARS element helpers ---
// Emit a struct field:  "Type Name {Default};"  or  "Type Name;"
#define HBS__VAR_STRUCT_FIELD(tuple_)        HBS__VAR_STRUCT_FIELD_I tuple_
#define HBS__VAR_STRUCT_FIELD_I(T, N, ...)   T N __VA_OPT__(= __VA_ARGS__);

// --- PARAMS element helpers ---

// Emit a struct field:  "Type Name {Default};"  or  "Type Name;"
#define HBS__PARAM_STRUCT_FIELD(tuple_)       HBS__PARAM_STRUCT_FIELD_I tuple_
#define HBS__PARAM_STRUCT_FIELD_I(T, N, ...)  T N __VA_OPT__(= __VA_ARGS__);

// New per-field stream helper (standalone: replace body with nothing)
#define HBS__PARAM_STREAM_FIELD(tuple_)      HBS__PARAM_STREAM_FIELD_I tuple_
#define HBS__PARAM_STREAM_FIELD_I(T, N, ...) Streaming::streamIt(cfg, #N, N);

// ===========================================================================
// Tagged optional-section scanning (used by CORO_DEFINE / HSM_DEFINE)
// ===========================================================================
//
// ARGS(...), VARS(...), DEFINES_PARAMS(...) each produce a tagged tuple:
//   ARGS(...)           -> (HBS_ARGS_TAG,   __VA_ARGS__)
//   VARS(...)           -> (HBS_VARS_TAG,   __VA_ARGS__)
//   DEFINES_PARAMS(...) -> (HBS_PARAMS_TAG, __VA_ARGS__)
//
// HBS__FIND_ARGS / HBS__FIND_VARS / HBS__FIND_PARAMS scan the variadic
// tail of CORO_DEFINE / HSM_DEFINE (up to 3 optional sections) and return
// the content of the matching tagged item, or nothing if absent.

// Per-tag content extractors.
// HBS__ITEM_IF_ARGS(ITEM): if ITEM carries HBS_ARGS_TAG, expand to its content.
#define HBS__ITEM_IF_ARGS(ITEM)          HBS__ITEM_IF_ARGS_I ITEM
#define HBS__ITEM_IF_ARGS_I(tag_, ...)   HBS__ITEM_IF_ARGS_II_##tag_(__VA_ARGS__)
#define HBS__ITEM_IF_ARGS_II_HBS_ARGS_TAG(...)   __VA_ARGS__
#define HBS__ITEM_IF_ARGS_II_HBS_VARS_TAG(...)   /* not args */
#define HBS__ITEM_IF_ARGS_II_HBS_PARAMS_TAG(...) /* not args */
#define HBS__ITEM_IF_ARGS_II_HBS_LPARAMS_TAG(...) /* not args */

#define HBS__ITEM_IF_VARS(ITEM)          HBS__ITEM_IF_VARS_I ITEM
#define HBS__ITEM_IF_VARS_I(tag_, ...)   HBS__ITEM_IF_VARS_II_##tag_(__VA_ARGS__)
#define HBS__ITEM_IF_VARS_II_HBS_ARGS_TAG(...)   /* not vars */
#define HBS__ITEM_IF_VARS_II_HBS_VARS_TAG(...)   __VA_ARGS__
#define HBS__ITEM_IF_VARS_II_HBS_PARAMS_TAG(...) /* not vars */
#define HBS__ITEM_IF_VARS_II_HBS_LPARAMS_TAG(...) /* not vars */

#define HBS__ITEM_IF_PARAMS(ITEM)          HBS__ITEM_IF_PARAMS_I ITEM
#define HBS__ITEM_IF_PARAMS_I(tag_, ...)   HBS__ITEM_IF_PARAMS_II_##tag_(__VA_ARGS__)
#define HBS__ITEM_IF_PARAMS_II_HBS_ARGS_TAG(...)   /* not params */
#define HBS__ITEM_IF_PARAMS_II_HBS_VARS_TAG(...)   /* not params */
#define HBS__ITEM_IF_PARAMS_II_HBS_PARAMS_TAG(...) __VA_ARGS__
#define HBS__ITEM_IF_PARAMS_II_HBS_LPARAMS_TAG(...) __VA_ARGS__ /* loads params can also define them */

#define HBS__ITEM_IF_LPARAMS(ITEM)          HBS__ITEM_IF_LPARAMS_I ITEM
#define HBS__ITEM_IF_LPARAMS_I(tag_, ...)   HBS__ITEM_IF_LPARAMS_II_##tag_(__VA_ARGS__)
#define HBS__ITEM_IF_LPARAMS_II_HBS_ARGS_TAG(...)   /* not loads params */
#define HBS__ITEM_IF_LPARAMS_II_HBS_VARS_TAG(...)   /* not loads params */
#define HBS__ITEM_IF_LPARAMS_II_HBS_PARAMS_TAG(...) /* not loads params */
#define HBS__ITEM_IF_LPARAMS_II_HBS_LPARAMS_TAG(...) __VA_ARGS__


// Scan up to 3 optional section arguments and collect matching content.
//
// Each HBS__ITEM_IF_*(ITEM) expands to the content of ITEM if the tag matches,
// or to nothing otherwise.  The results are space-separated (no commas), so
// the output is a sequence of zero or more tuple tokens suitable for HBS__MAP.
//
// We use a fixed-arity approach (no recursion, which the preprocessor forbids).
// Up to 3 optional sections are supported; extras are silently ignored.
//
// Padding: we pad with (HBS__EMPTY_TAG,) which all three extractors ignore.
// HBS__EMPTY_TAG is a dedicated tag that all HBS__ITEM_IF_*_II_ dispatchers
// map to nothing.

#define HBS__ITEM_IF_ARGS_II_HBS__EMPTY_TAG(...)   /* empty slot */
#define HBS__ITEM_IF_VARS_II_HBS__EMPTY_TAG(...)   /* empty slot */
#define HBS__ITEM_IF_PARAMS_II_HBS__EMPTY_TAG(...) /* empty slot */
#define HBS__ITEM_IF_LPARAMS_II_HBS__EMPTY_TAG(...) /* empty slot */

#define HBS__FIND_ARGS(...) \
    HBS__FIND_ARGS_I(__VA_ARGS__ __VA_OPT__(,) (HBS__EMPTY_TAG,), (HBS__EMPTY_TAG,), (HBS__EMPTY_TAG,))
#define HBS__FIND_ARGS_I(a_, b_, c_, ...) \
    HBS__ITEM_IF_ARGS(a_) HBS__ITEM_IF_ARGS(b_) HBS__ITEM_IF_ARGS(c_)

#define HBS__FIND_VARS(...) \
    HBS__FIND_VARS_I(__VA_ARGS__ __VA_OPT__(,) (HBS__EMPTY_TAG,), (HBS__EMPTY_TAG,), (HBS__EMPTY_TAG,))
#define HBS__FIND_VARS_I(a_, b_, c_, ...) \
    HBS__ITEM_IF_VARS(a_) HBS__ITEM_IF_VARS(b_) HBS__ITEM_IF_VARS(c_)

#define HBS__FIND_PARAMS(...) \
    HBS__FIND_PARAMS_I(__VA_ARGS__ __VA_OPT__(,) (HBS__EMPTY_TAG,), (HBS__EMPTY_TAG,), (HBS__EMPTY_TAG,))
#define HBS__FIND_PARAMS_I(a_, b_, c_, ...) \
    HBS__ITEM_IF_PARAMS(a_) HBS__ITEM_IF_PARAMS(b_) HBS__ITEM_IF_PARAMS(c_)

#define HBS__FIND_LPARAMS(...) \
    HBS__FIND_LPARAMS_I(__VA_ARGS__ __VA_OPT__(,) (HBS__EMPTY_TAG,), (HBS__EMPTY_TAG,), (HBS__EMPTY_TAG,))
#define HBS__FIND_LPARAMS_I(a_, b_, c_, ...) \
    HBS__ITEM_IF_LPARAMS(a_) HBS__ITEM_IF_LPARAMS(b_) HBS__ITEM_IF_LPARAMS(c_)    

// HBS__MAP_ARGS/VARS/PARAMS: scan the section list for the matching tag,
// then map the helper macro over the found elements.
//
// The two-level indirection (_I suffix) forces HBS__FIND_* to expand fully
// before HBS__MAP counts its arguments.  Because HBS__MAP_ARGS_I is variadic,
// the preprocessor expands HBS__FIND_ARGS(sections...) during argument
// collection, so __VA_ARGS__ inside _I contains the already-expanded list.

#define HBS__MAP_ARGS(M, D, ...)   HBS__MAP_ARGS_I(M, D, HBS__FIND_ARGS(__VA_ARGS__))
#define HBS__MAP_ARGS_I(M, D, ...) HBS__MAPC(M, D __VA_OPT__(,) __VA_ARGS__)

#define HBS__MAP_VARS(M, D, ...)   HBS__MAP_VARS_I(M, D, HBS__FIND_VARS(__VA_ARGS__))
#define HBS__MAP_VARS_I(M, D, ...) HBS__MAPC(M, D __VA_OPT__(,) __VA_ARGS__)

#define HBS__MAP_PARAMS(M, D, ...)   HBS__MAP_PARAMS_I(M, D, HBS__FIND_PARAMS(__VA_ARGS__))
#define HBS__MAP_PARAMS_I(M, D, ...) HBS__MAPC(M, D __VA_OPT__(,) __VA_ARGS__)

#define HBS__MAP_LPARAMS(M, D, ...)   HBS__MAP_LPARAMS_I(M, D, HBS__FIND_LPARAMS(__VA_ARGS__))
#define HBS__MAP_LPARAMS_I(M, D, ...) HBS__MAPC(M, D __VA_OPT__(,) __VA_ARGS__)

// special handling to only emit loadParams body if appropriate

#define HBS__HAS_LPARAMS(...) HBS__HAS_LPARAMS_I(HBS__FIND_LPARAMS(__VA_ARGS__))
#define HBS__HAS_LPARAMS_I(...) __VA_OPT__(1)

#define HBS__PARAMS_LOAD_CONFIG(hasLoad_, ...) \
    HBS__PARAMS_LOAD_CONFIG_I(hasLoad_, __VA_ARGS__)
#define HBS__PARAMS_LOAD_CONFIG_I(hasLoad_, ...) \
    HBS__PARAMS_LOAD_CONFIG_##hasLoad_(__VA_ARGS__)

// LOADS_PARAMS case — real file open + assert + stream
#define HBS__PARAMS_LOAD_CONFIG_1(...) \
  void loadConfigFile(const std::string& prefix, const std::string& name) \
  { \
    Hbs::ConfigFile cfg(prefix, name); \
    HBS__MAP_LPARAMS(HBS__PARAM_STREAM_FIELD, , __VA_ARGS__) \
  }

// PARAMS case — no file, no call overhead
#define HBS__PARAMS_LOAD_CONFIG_(...) \
  void loadConfigFile(const std::string&, const std::string&) {}


// ===========================================================================
// Inline vs out-of-class DEFINE dispatch
// ===========================================================================
//
// CORO_DEFINE((taskName), ...)          — inline (1-element name tuple)
// CORO_DEFINE((ClassName, taskName), .) — out-of-class (2-element name tuple)
//
// Detection: probe for a second element using __VA_OPT__
//   HBS__HAS_CLASS(a, ...) expands to 1 when a second element is present.

#define HBS__HAS_CLASS(a_, ...) __VA_OPT__(1)

// Extract class name (first) and task name (second) from a 2-element tuple.
#define HBS__TUPLE_CLASS(a_, b_) a_
#define HBS__TUPLE_TASK(a_, b_)  b_
// Extract the sole name from a 1-element tuple.
#define HBS__TUPLE_ONLY(a_)      a_

// HBS__DEFINE_DISPATCH — generic inline-vs-out-of-class router.
//
// PREFIX_ is the macro-name prefix (e.g. HBS__CORO or HBS__HSM).
// The challenge: two separate expansions must happen before token-pasting:
//   1. HBS__HAS_CLASS NAMETUPLE_ must expand to empty or "1"
//   2. HBS__TUPLE_ONLY / HBS__TUPLE_CLASS / HBS__TUPLE_TASK must strip parens
//      from NAMETUPLE_ before the result is passed to the INLINE/OUTOFCLASS macro
//      (otherwise taskName_ receives "(moveTask)" and ## produces ")_ctx" etc.)
//
// Solution: add a _NAME_EXPAND level that forces the tuple-stripping macros to
// expand before their results are forwarded to the body macro.

// Step 1: evaluate HBS__HAS_CLASS on the tuple (function-like application)
#define HBS__DEFINE_DISPATCH(PREFIX_, NAMETUPLE_, ...)                                \
    HBS__DEFINE_DISPATCH_I(PREFIX_, HBS__HAS_CLASS NAMETUPLE_, NAMETUPLE_, __VA_ARGS__)
// Step 2: hasClass_ is now fully expanded; forward to paste step
#define HBS__DEFINE_DISPATCH_I(PREFIX_, hasClass_, ...)                               \
    HBS__DEFINE_DISPATCH_II(PREFIX_, hasClass_, __VA_ARGS__)
// Step 3: paste to select _PICK_ or _PICK_1
#define HBS__DEFINE_DISPATCH_II(PREFIX_, hasClass_, NAMETUPLE_, ...)                  \
    HBS__DEFINE_PICK_##hasClass_(PREFIX_, NAMETUPLE_, __VA_ARGS__)

// 1-element tuple -> inline.
// HBS__TUPLE_ONLY NAMETUPLE_ is a function-like call that strips parens.
// We need one more level so the stripped name is expanded before ## in the body.
#define HBS__DEFINE_PICK_(PREFIX_, NAMETUPLE_, ...)                                   \
    HBS__DEFINE_INLINE_NAMED(PREFIX_, HBS__TUPLE_ONLY NAMETUPLE_, __VA_ARGS__)
#define HBS__DEFINE_INLINE_NAMED(PREFIX_, taskName_, ...)                             \
    PREFIX_##_DEFINE_INLINE(taskName_, __VA_ARGS__)

// 2-element tuple -> out-of-class.
#define HBS__DEFINE_PICK_1(PREFIX_, NAMETUPLE_, ...)                                  \
    HBS__DEFINE_OUTOFCLASS_NAMED(PREFIX_, HBS__TUPLE_CLASS NAMETUPLE_, HBS__TUPLE_TASK NAMETUPLE_, __VA_ARGS__)
#define HBS__DEFINE_OUTOFCLASS_NAMED(PREFIX_, className_, taskName_, ...)             \
    PREFIX_##_DEFINE_OUTOFCLASS(className_, taskName_, __VA_ARGS__)

// CORO and HSM thin wrappers — each just calls the generic dispatcher with its prefix.
#define HBS__CORO_DEFINE_DISPATCH(NAMETUPLE_, ...) \
    HBS__DEFINE_DISPATCH(HBS__CORO, NAMETUPLE_, __VA_ARGS__)
#define HBS__HSM_DEFINE_DISPATCH(NAMETUPLE_, ...) \
    HBS__DEFINE_DISPATCH(HBS__HSM, NAMETUPLE_, __VA_ARGS__)

