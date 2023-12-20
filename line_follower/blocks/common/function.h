// Copyright (c) 2020 Adam Ryczkowski

#ifndef LINE_FOLLOWER_BLOCKS_COMMON_FUNCTION_H_
#define LINE_FOLLOWER_BLOCKS_COMMON_FUNCTION_H_

namespace line_follower
{

namespace detail
{

template<typename Result,typename ...Args>
struct abstract_function
{
    virtual Result operator()(Args... args)=0;
    virtual abstract_function *clone() const =0;
    virtual ~abstract_function() = default;
};

template<typename Func,typename Result,typename ...Args>
class concrete_function: public abstract_function<Result,Args...>
{
    Func f;
public:
    concrete_function(const Func &x)
        : f(x)
    {}
    Result operator()(Args... args) override
    {
        return f(args...);
    }
    concrete_function *clone() const override
    {
        return new concrete_function{f};
    }
};

template<typename Func>
struct func_filter
{
    typedef Func type;
};
template<typename Result,typename ...Args>
struct func_filter<Result(Args...)>
{
    typedef Result (*type)(Args...);
};

}  // namespace detail

template<typename signature>
class FunctionObject;

template<typename Result,typename ...Args>
class FunctionObject<Result(Args...)>
{
    detail::abstract_function<Result,Args...> *f;
public:
    FunctionObject()
        : f(nullptr)
    {}
    template<typename Func> FunctionObject(const Func &x)
        : f(new detail::concrete_function<typename detail::func_filter<Func>::type,Result,Args...>(x))
    {}
    FunctionObject(const FunctionObject &rhs)
        : f(rhs.f ? rhs.f->clone() : nullptr)
    {}
    FunctionObject &operator=(const FunctionObject &rhs)
    {
        if( (&rhs != this ) && (rhs.f) )
        {
            auto *temp = rhs.f->clone();
            delete f;
            f = temp;
        }
        return *this;
    }
    template<typename Func> FunctionObject &operator=(const Func &x)
    {
        auto *temp = new detail::concrete_function<typename detail::func_filter<Func>::type,Result,Args...>(x);
        delete f;
        f = temp;
        return *this;
    }
    Result operator()(Args... args)
    {
        if(f)
            return (*f)(args...);
        else
            return Result();
    }
    ~FunctionObject()
    {
        delete f;
    }
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_COMMON_FUNCTION_H_