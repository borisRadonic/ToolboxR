#pragma once
#include <cstdint>

namespace StaticMemAllocHelper
{

    /**
    * @brief Memory storage template for pre-allocation.
    *
    * This template is designed to pre-allocate memory for objects on the stack.
    *
    * @tparam T Type of the elements the storage will manage.
    * @tparam n Number of elements of type T the storage will manage. Default is 1.
    */
    template<typename T, size_t n = 1U>
    struct alignas(sizeof(size_t)) PREALLOC_STATIC_MEMORY
    {
    public:

        /**
        * @brief Default constructor.
        */
        PREALLOC_STATIC_MEMORY() noexcept
        {
        }

       

    public:

        /**
        *@brief Retrieves the pointer to the beginning of the memory block.
        *
        * @return Void pointer to the beginning of the memory block.
        */
        inline void* get_ptr() noexcept
        {
            return (void*)_memory;
        }

        /**
        * @brief Retrieves the pointer to the next available memory block.
        *
        * It increments the internal count to keep track of utilized memory blocks.
        *
        * @return Void pointer to the next available memory block, or nullptr if no block is available.
        */
        inline void* get_next_ptr() noexcept
        {
            if (_count < n)
            {
                void* ret = (void*)(_memory + _count * sizeof(T));
                _count++;
                return ret;
            }
            return nullptr;
        }
    private:
        uint8_t _memory[n * sizeof(T)]; ///< Pre-allocated memory storage.
        size_t _count = 0U;             ///< Counter for used blocks in the storage.
    };

};

