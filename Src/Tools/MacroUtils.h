#pragma once

// see https://www.mikeash.com/pyblog/friday-qa-2015-03-20-preprocessor-abuse-and-optional-parentheses.html
#define M_EXTRACT_ARGS_(...) M_EXTRACT_ARGS_ __VA_ARGS__
#define DO_NOT_M_EXTRACT_ARGS_
#define M_CONCAT_(x_, ...) x_ ## __VA_ARGS__
#define M_EVAL_BEFORE_CONCAT_(x_, ...) M_CONCAT_(x_, __VA_ARGS__)

// remove one level of parentheses that may or may not be present
#define M_REMOVE_OPTIONAL_PAREN(x_) M_EVAL_BEFORE_CONCAT_(DO_NOT_, M_EXTRACT_ARGS_ x)

// remove one level of parentheses that must be present
#define M_REMOVE_PAREN(x_) x_


// help concatenating two elements, particularly where one of them needs
// an extra level of evaluation so that it can be concatenated properly, e.g. __LINE__

#define M_CONCAT_LINE(first)      M_CONCAT2(first,__LINE__)
#define M_CONCAT2(first, second)   M_CONCAT_SIMPLE(first, second)
#define M_CONCAT_SIMPLE(first, second) first##second

// evaluating macros before use

#define M_EVAL(x_) M_EVAL2(x_)
#define M_EVAL2(x_) x_

#define M_STRINGIZE(x_) M_STRINGIZE2(x_)
#define M_STRINGIZE2(x_) #x_

#define M_MESSAGE(x_) #x_ " is " M_STRINGIZE(x_)