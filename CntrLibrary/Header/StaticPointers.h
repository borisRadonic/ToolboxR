#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <memory>

#include "StaticMemAllocHelper.h"

namespace StaticPointers
{
    /**
    * @brief A smart pointer that uniquely owns an object created using placement new in pre-allocated static memory.
    *
    * @tparam T The type of object to manage.
    * @tparam n The number of elements of type T the static memory will manage. Default is 1.
    */
    template <typename T>
    class StaticMemUniquePtr
    {
    public:
       /**
        * @brief Default constructor.
        *
        * @param aPtr Optional pointer to initialize the managed object.
        */
        template <typename... Args>
        explicit StaticMemUniquePtr(T* aPtr = nullptr): ptr(aPtr)
        {           
        }

        /**
        * @brief Constructs an object of type T in the pre-allocated static memory.
        *
        * @tparam Args Types of the arguments to pass to the constructor of T.
        * @param args Arguments to pass to the constructor of T.
        * 
        * @throws std::bad_alloc If there is no available memory.
        */
        template <typename... Args>
        void construct(Args&&... args)
        {
            void* memory = allocator.get_next_ptr();           
            ptr = new (memory) T(std::forward<Args>(args)...);       
        }

        /**
        * @brief Destroys the managed object if it exists.
        */
        ~StaticMemUniquePtr()
        {
            if (ptr)
            {
                ptr->~T();
            }
        }

        StaticMemUniquePtr(const StaticMemUniquePtr&) = delete;
        StaticMemUniquePtr& operator=(const StaticMemUniquePtr&) = delete;

        /**
        * @brief Move constructor. Transfers ownership from another StaticMemUniquePtr.
        * 
        * @param other The StaticMemUniquePtr to move from.
        */
        StaticMemUniquePtr(StaticMemUniquePtr&& other) noexcept
            : ptr(other.ptr)
        {
            other.ptr = nullptr;
        }

        /**
        * @brief Move assignment operator. Transfers ownership from another StaticMemUniquePtr.
        *
        * @param other The StaticMemUniquePtr to move from.
        * @return Reference to this StaticMemUniquePtr.
        */
        StaticMemUniquePtr& operator=(StaticMemUniquePtr&& other) noexcept
        {
            if (this != &other)
            {
                reset();
                this->ptr = other.ptr;
                other.ptr = nullptr;
            }
            return *this;
        }

        /**
        * @brief Dereferences the managed object.
        *
        * @return Reference to the managed object.
        */
        T& operator*() const
        {
            return *ptr;
        }

        /**
        * @brief Accesses a member of the managed object.
        *
        * @return Pointer to the managed object.
        */
        T* operator->() const
        {
            return ptr;
        }

        /**
        * @brief Gets the raw pointer to the managed object.
        *
        * @return Pointer to the managed object.
        */
        T* get() const
        {
            return ptr;
        }

        /**
        * @brief Releases ownership of the managed object without destroying it.
        *
        * @return Pointer to the managed object.
        */
        T* release()
        {
            T* temp = ptr;
            ptr = nullptr;
            return temp;
        }

        /**
        * @brief Destroys the managed object and resets the pointer.
        */
        void reset()
        {
            if (ptr)
            {
                ptr->~T();
                ptr = nullptr;
            }
        }

    private:
        T* ptr = nullptr; ///< Pointer to the managed object.
        StaticMemAllocHelper::PREALLOC_STATIC_MEMORY<T> allocator; ///< Static memory allocator.
    };

    // StaticSharedPtr implementation without reference counting
    template <typename T>
    class StaticSharedPtr
    {
    public:
        // Default constructor
        template <typename... Args>
        explicit StaticSharedPtr(T* aPtr = nullptr) : ptr(aPtr)
        {
        }

        // Constructs an object of type T in the pre-allocated static memory.
        template <typename... Args>
        void construct(Args&&... args)
        {
            void* memory = allocator.get_next_ptr();
            ptr = new(memory) T(std::forward<Args>(args)...);
        }

        // Destructor
        ~StaticSharedPtr()
        {
            reset();
        }

        // Copy constructor (deleted to prevent copying)
        StaticSharedPtr(const StaticSharedPtr&) = delete;

        // Copy assignment operator (deleted to prevent copying)
        StaticSharedPtr& operator=(const StaticSharedPtr&) = delete;

        // Move constructor (transfers ownership)
        StaticSharedPtr(StaticSharedPtr&& other) noexcept : ptr(other.ptr)
        {
            other.ptr = nullptr;
        }

        // Move assignment operator (transfers ownership)
        StaticSharedPtr& operator=(StaticSharedPtr&& other) noexcept
        {
            if (this != &other)
            {
                reset();
                ptr = other.ptr;
                other.ptr = nullptr;
            }
            return *this;
        }

        T& operator*() const
        {
            return *ptr;
        }

        T* operator->() const {
            return ptr;
        }

        T* get() const
        {
            return ptr;
        }

        void reset()
        {
            if (ptr)
            {
                ptr->~T();
                ptr = nullptr;
            }
        }

    private:
        T* ptr = nullptr;
        StaticMemAllocHelper::PREALLOC_STATIC_MEMORY<T> allocator;
    };
}
