#pragma once

#include "StaticPointers.h"
#include <memory>
#include <type_traits>

namespace FlexPointers
{
    // Specialized template for FlexiblePtr (dynamic allocation)
    template <typename T, bool UseStatic = false>
    class FlexUniquePtr
    {
    public:
        template <typename... Args>
        void create(Args&&... args)
        {
            dynamicPtr = std::make_unique<T>(std::forward<Args>(args)...);
        }

        T& operator*() const
        {
            return *dynamicPtr;
        }

        T* operator->() const
        {
            return dynamicPtr.get();
        }

    private:
        std::unique_ptr<T> dynamicPtr;
    };

    // Specialized template for FlexiblePtr (static allocation)
    template <typename T>
    class FlexUniquePtr<T, true>
    {
    public:
        template <typename... Args>
        void create(Args&&... args)
        {
            staticPtr.construct(std::forward<Args>(args)...);
        }

        T& operator*() const
        {
            return *staticPtr;
        }

        T* operator->() const
        {
            return staticPtr.get();
        }

    private:
        StaticPointers::StaticMemUniquePtr<T> staticPtr;
    };


    // Primary template for FlexibleSharedPtr (dynamic allocation using std::shared_ptr)
    template <typename T, bool UseStatic = false>
    class FlexibleSharedPtr
    {
    public:

        template <typename... Args>
        void create(Args&&... args)
        {
            dynamicPtr = std::make_shared<T>(std::forward<Args>(args)...);
        }

        T& operator*() const
        {
            return *dynamicPtr;
        }

        T* operator->() const
        {
            return dynamicPtr.get();
        }

        int use_count() const
        {
            return dynamicPtr.use_count();
        }

    private:
        std::shared_ptr<T> dynamicPtr;
    };


    // Specialized template for FlexibleSharedPtr (static allocation using StaticSharedPtr)
    template <typename T>
    class FlexibleSharedPtr<T, true>
    {
    public:

        template <typename... Args>
        void create(Args&&... args)
        {
            staticPtr.construct(std::forward<Args>(args)...);
        }

        T& operator*() const
        {
            return *staticPtr;
        }

        T* operator->() const
        {
            return staticPtr.get();
        }

    private:
        StaticPointers::StaticSharedPtr<T> staticPtr;
    };
}