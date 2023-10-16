/**
 * @file: MacroUtils.h
 * 
 * helper code for macros - centralized here to avoid unnecessary duplication
 * 
 * @author: Rudi Villing
 */

#pragma once

/** Concatenate two parameters - multiple levels of expansion to ensure parameters can be joined. */
#define _JOIN(a, b) _JOIN_I(a, b)
#define _JOIN_I(a, b) _JOIN_II(a ## b)
#define _JOIN_II(res) res


/**
 * Determine the number of args to a macro (only designed for small numbers of arguments).
 * 
 * Example:
 *       _NARGS(a, b, c)
 * -->   _NARGS_I(a,b,c, 5,4,3,2,1,0)
 * -->   _NARGS_II(a,b,c, 5,4,3,2,1,0)
 * -->   3
 */
#define _NARGS(...)       _NARGS_I(__VA_ARGS__, _NARGS_III)
#define _NARGS_I(...)     _NARGS_II(__VA_ARGS__)
#define _NARGS_II(_0, _1, _2, _3, _4, N, ...)  N
#define _NARGS_III        5, 4, 3, 2, 1, 0

/**
 * support overloading of macros with different numbers of parameters
 * 
 * Automatically redirect to the appropriate overloaded macro based
 * on the base macro name (prefix) and arg list.
 * 
 * The overloaded macros are given by the prefix followed by _ and the number of params
 * 
 * Declare the actual overrides with the appropriate number of params to match their name
 * 
 * WARNING --- does not work for zero params!
 * 
 * Example: 
 * 
 *    #define SOME_MACRO(...) CHOOSE_FUNC(SOME_MACRO, __VA_ARGS__)
 * 
 *    // add support for 1, or 2 argument versions of SOME_MACRO 
 *    #define SOME_MACRO_1(a)   fillInTheCodeHere...
 *    #define SOME_MACRO_2(a,b) fillInTheCodeHere...
 * 
 * @param f the base name/prefix to be used for the macros to be selected
 * 
 * see: https://stackoverflow.com/questions/11761703/overloading-macro-on-number-of-arguments
 */
#define CHOOSE_FUNC(f, ...)     _CHOOSE_FUNC_I(f, _NARGS(__VA_ARGS__)) (__VA_ARGS__)
#define _CHOOSE_FUNC_I(name, n) _CHOOSE_FUNC_II(name, n)
#define _CHOOSE_FUNC_II(name, n)  name##_##n
