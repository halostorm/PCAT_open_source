#ifndef FACTORY_TYPE_UTILS_H
#define FACTORY_TYPE_UTILS_H

#include <tuple>
#include <string>
#include <string_view>

/**
 * @brief get type name from template arg
 * @example
 * Type<int>::name      -> constexpr std::string_view("int")
 * Type<int>::str()     -> std::string("int")
 * Type<int>::c_str()   -> std::string("int").c_str()
 */
template<typename T>
struct Type {
private:
    static constexpr auto type_name_impl() noexcept {
        std::string_view name, prefix, suffix;
#ifdef __clang__
        name = __PRETTY_FUNCTION__;
    prefix = "static auto Type<T>::type_name_impl() [T = ";
    suffix = "]";
#elif defined(__GNUC__)
        name = __PRETTY_FUNCTION__;
        prefix = "static constexpr auto Type<T>::type_name_impl() [with T = ";
        suffix = "]";
#elif defined(_MSC_VER)
        name = __FUNCSIG__;
    prefix = "static auto __cdecl Type<T>::type_name_impl<";
    suffix = ">(void) noexcept";
#endif
        name.remove_prefix(prefix.size());
        name.remove_suffix(suffix.size());
        return name;
    }

public:
    static constexpr auto name = type_name_impl();

    static std::string &str() {
        static std::string n(name);
        return n;
    }

    static auto *c_str() { return str().c_str(); }
};

/**
 * @brief get type name from template args
 * @example
 * Types<int,double>::name<1>   -> constexpr std::string_view("double")
 * Types<int,double>::names     -> constexpr std::string_view("<int, double>")
 * Types<int,double>::raw_names -> constexpr std::string_view("int, double")
 */
template<typename ...T>
struct Types {
private:
    static constexpr auto type_names_impl() noexcept {
        std::string_view name = Type<Types<T...>>::name, prefix = "Types";
        name.remove_prefix(prefix.size());
        return name;
    }

    static constexpr auto raw_type_names_impl() noexcept {
        std::string_view name = type_names_impl(), prefix = "<", suffix = ">";
        name.remove_prefix(prefix.size());
        name.remove_suffix(suffix.size());
        return name;
    }

public:

    static constexpr size_t size = sizeof...(T);
    template<size_t N>
    static constexpr auto name = Type<typename std::tuple_element<N, std::tuple<T...> >::type>::name;

    static constexpr auto names = type_names_impl();
    static constexpr auto raw_names = raw_type_names_impl();
};

/**
 * @brief decorate template class name with template arg
 * @example
 * DECORATE(TemplateClass,int,double)  -> std::string("TemplateClass<int, double>")
 * decorate<int,double>(TemplateClass) -> std::string("TemplateClass<int, double>")
 */
#define DECORATE(name, ...) decorate<__VA_ARGS__>(name)

template<class  ...T>
inline std::string decorate(const std::string &name) { return name + std::string(Types<T...>::names); }

template<>
inline std::string decorate(const std::string &name) { return name; }

/**
 * @brief template class type trait
 * @example
 * TTypeTrait<TemplateClass<int,double>>::size                          -> size_t(2)
 * TTypeTrait<TemplateClass<int,double>>::arg_type<1>                   -> double
 * TTypeTrait<TemplateClass<int,double>>::with_template_arg(NewClass)   -> std::string("NewClass<int, double>")
 */

template<class T>
struct TTypeTrait {
    constexpr static size_t size = 0;

    static std::string with_template_arg(const std::string &name) { return name; }
};

template<template<class> class TType, class ...T>
struct TTypeTrait<TType<T...>> {
private:
    template<size_t ...N>
    static std::string
    with_template_arg_helper(const std::string &name, std::index_sequence<N...>) {
        return decorate<arg_type < N>...>(name);
    }

public:
    constexpr static size_t size = sizeof ...(T);
    template<size_t N>
    using arg_type = typename std::tuple_element<N, std::tuple<T...>>::type;

    static std::string with_template_arg(const std::string &name) {
        return with_template_arg_helper(name, std::make_index_sequence<size>());
    }
};

/***
 * @brief SFINAE to detect if CLASS have static function called "name"
 * @example
 * struct A {
 * static void name() {}
 * };
 * struct B {}
 *
 * has_member_name_v<A>  ->  constexpr bool(true)
 * has_member_name_v<B>  ->  constexpr bool(false)
 */
#define PC_UTILS_HAS_MEMBER(XXX) \
template<typename T, typename... Args>\
struct has_member_##XXX \
{ \
private:  \
  template<typename U> static auto Check(int) -> decltype(std::declval<U>().XXX(std::declval<Args>()...), std::true_type());  \
  template<typename U> static std::false_type Check(...); \
public: \
  static constexpr auto value = decltype(Check<T>(0))::value; \
};\
template <class T> inline constexpr bool has_member_##XXX##_v{has_member_##XXX<T>::value};

PC_UTILS_HAS_MEMBER(name)

/***
 * @brief get unique symbol
 * @example
 * REGISTER_TO_FACTORY_UNIQUE_ID(ABC) ABC1
 * REGISTER_TO_FACTORY_UNIQUE_ID(ABC) ABC2
 */
#define REGISTER_TO_FACTORY_UNIQUE_ID_MERGE_IMPL(a, b) a ## b //合并用的主体
#define REGISTER_TO_FACTORY_UNIQUE_ID_MERGE(a, b) REGISTER_TO_FACTORY_UNIQUE_ID_MERGE_IMPL(a, b) //中间层
#define REGISTER_TO_FACTORY_UNIQUE_ID(name) REGISTER_TO_FACTORY_UNIQUE_ID_MERGE(name, __COUNTER__)

#define REGISTER_REMOVE_PARENTHESES_IMPL(X) X
#define REGISTER_REMOVE_PARENTHESES(X)  REGISTER_REMOVE_PARENTHESES_IMPL X

/***
 * @brief define namespace addition
 */

#define REGISTER_DEF_NAMESPACE_DECORATE                     inline std::string ns(const std::string &name) {            \
                                                                struct Ns {                                             \
                                                                    static constexpr auto name_space() noexcept {       \
                                                                    auto name = Type<Ns>::name,                         \
                                                                    suffix = std::string_view("ns(const string&)::Ns"); \
                                                                    name.remove_suffix(suffix.size());                  \
                                                                    return name;                                        \
                                                                    }                                                   \
                                                                };                                                      \
                                                                return std::string(Ns::name_space()) + name;            \
                                                            }

#endif