#ifndef FACTORY_H
#define FACTORY_H

#include "type_util.h"
#include <memory>
#include <unordered_map>
//#define BASE_UTILS_VERBOSE
/***
 * @brief Factory class definition
 * @example
 * struct Base{}
 * struct Der{ Der(int){}}
 * static bool register_1 = Factory<Base,int>::Register<Der>();
 */
template<class Base, class ...Args>
struct Factory {
private:
    template<class ...T>
    static auto *BuildImpl(const std::string &name, T &&...t) {
        if (GetTable().find(name) == GetTable().end()) {
            fprintf(stderr, "build fail: %s no exist in %s.\n", name.c_str(),
                    Type<Factory<Base, Args...>>::c_str());
            return (Base *) nullptr;
        } else {
            return GetTable()[name](std::forward<T>(t)...);
        }
    }

    static inline auto &GetTable() {
        static std::unordered_map<std::string, Base *(*)(Args...)> table;
        return table;
    }

    template<class>
    struct raw_ptr;
public:

    template<class Der>
    static bool Register() {
        constexpr bool validation = std::is_default_constructible_v<Der> || std::is_constructible_v<Der, Args...>;
        static_assert(validation, "no Der(Args...) constructor function exist\n");
        if constexpr (validation) {
            static_assert(std::is_base_of_v<Base, Der>, "no derivative relationship between this two types.\n");
            std::string hash_code;
            if constexpr (has_member_name_v<Der>)
                hash_code = std::string(Der::name());
            else
                hash_code = std::string(Type<Der>::name);

            if (GetTable().find(hash_code) == GetTable().end()) {
#ifdef BASE_UTILS_VERBOSE
                printf("register [√]: %s -> %s\n",
                       Type<Der>::c_str(),
                       Type<Factory<Base, Args...>>::c_str());
#endif
                GetTable()[hash_code] = [](Args ...args) -> Base * { return new Der(args...); };
            } else {
                fprintf(stderr, "register [x]:  %s  ->  %s\n",
                        Type<Der>::c_str(),
                        Type<Factory<Base, Args...>>::c_str());
            }
            return true;
        } else
            return false;
    }

    template<template<class> class PtrType=raw_ptr, class ...T>
    static auto Build(const std::string &name, T &&...t) {
        if constexpr (std::is_same_v<PtrType<Base>, raw_ptr<Base>>) return BuildImpl(name, std::forward<T>(t)...);
        else return PtrType<Base>(BuildImpl(name, std::forward<T>(t)...));
    }

    template<template<class> class PtrType=std::shared_ptr, class ...T>
    static auto BuildT(const std::string &name, T &&...t) {
        return Build<PtrType>(TTypeTrait<Base>::with_template_arg(name), std::forward<T>(t)...);
    }
};

/***
 * @brief auto register (support construction overload)
 * @example
 * struct Base{}
 * struct Der:  AutoRegister<Base, Der>::template OverLoad<>,
 *              AutoRegister<Base, Der>::template OverLoad<int>{
 *      Der(){}
 *      Der(int){}
 * }
 */
template<class Base, class Der>
struct AutoRegister {
    template<class ...Args>
    struct OverLoad {
        OverLoad() { (void) registered; /*必不可少，显式调用才会生成代码*/}

        static bool registered;
    };

    struct Default {
        Default() {
            (void) registered;
            Factory<Base>::template Register<Der>(); /*必不可少，显式调用才会生成代码*/}

        static bool registered;
    };
};
template<class Base, class Der>
template<class ...Args>
bool AutoRegister<Base, Der>::OverLoad<Args...>::registered = Factory<Base, Args...>::template Register<Der>();

template<class Base, class Der>
bool AutoRegister<Base, Der>::Default::registered = Factory<Base>::template Register<Der>();



/***
 * @brief for manually register
 * @example
 * struct Base{}
 * struct Der{
 *      Der(){}
 *      Der(int){}
 * }
 * REGISTER_TO_FACTORY(Base,Der)
 * REGISTER_TO_FACTORY(Base,Der, int)
 */


#define REGISTER_TO_FACTORY(der, ...) \
namespace{static bool REGISTER_TO_FACTORY_UNIQUE_ID(register_der) = Factory<__VA_ARGS__>::Register<REGISTER_REMOVE_PARENTHESES(der)>();}

/***
 * @brief define namespace decoration
 */
namespace pc_utils { REGISTER_DEF_NAMESPACE_DECORATE}


#endif