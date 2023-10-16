/**
 * The file declares a ring buffer. The type of the elements must be assignable and
 * copyable. They need to provide a default constructor if push_back or push_front
 * will be used without specifying a value. The interface of the
 * class is similar to types of the standard template library and it also supports
 * for-each loops.
 * 
 * Implementation detail:
 * 
 * The storage and sense of where the front and back of the buffer is are unusual, e.g.
 * If the characters A,B,C,D,E,F have been pushed to front, and then pop_back is called 3 times
 * the storage (With capacity 8) would look something like
 *
 * indexes   0   1   2   3   4   5   6   7  
 *         | - | - | - | D | E | F | - | - |    // size()=3
 *                       ^       ^   ^
 *                       |       |   +  head = 6
 *                       |       +-- front() = head - 1
 *                       +---------- back() = head - size()
 * 
 * calling push_front(G), push_back(H) would result in the storage looking as follows:
 *
 * indexes   0   1   2   3   4   5   6   7  
 *         | - | - | H | D | E | F | G | - |    // size()=5
 *                   ^               ^   ^
 *                   |               |   +-- head = 7
 *                   |               +-- front() = head
 *                   +------------------ back() = head - size()
 * 
 * It should be clear that push_front adds an element at the head and then 
 * increments it (wrapping if needed).
 * On the other hand, push_back increments the number of entries
 * and adds elements at head-size(). These locations are slightly 
 * counter-intuitive to the front and back in the function names.
 * 
 * 
 * @author Thomas Röfer
 * @author Rudi Villing
 */

#pragma once

#include <cstddef>
#include <cstring>
#include <iterator>
#include "Platform/BHAssert.h"
#include "Platform/Memory.h"

#include "fmt/format.h"

template<typename T, std::size_t n = 0> class RingBuffer
{
private:
  T* buffer; /**< Stores the elements of the buffer. */
  std::size_t allocated; /**< The capacity of the buffer. */
  std::size_t head = 0; /**< The next entry that will be used for push_front(). */
  std::size_t entries = 0; /**< The number of entries in the buffer. */

public:
  /** A class for iterators with its typical interface. */
  class iterator : public std::iterator<std::forward_iterator_tag, T>
  {
  private:
    RingBuffer<T, n>& buffer; /**< The buffer. */
    std::size_t index; /**< Index of the current entry. */

  public:
    iterator(RingBuffer<T, n>& buffer, std::size_t index) : buffer(buffer), index(index) {}
    iterator(const iterator& other) : iterator(other.buffer, other.index) {}
    iterator& operator=(const iterator& other) {return *new(this) iterator(other.buffer, other.index);}
    T* operator->() const {return &buffer[index];}
    T& operator*() const {return buffer[index];}
    bool operator==(const iterator& other) const {return index == other.index && &buffer == &other.buffer;}
    bool operator!=(const iterator& other) const {return index != other.index || &buffer != &other.buffer;}
    iterator& operator++() {++index; return *this;}
    iterator operator++(int) {iterator result(*this); ++index; return result;}
    iterator& operator+=(std::ptrdiff_t offset) {index += offset; return *this;}
    iterator& operator-=(std::ptrdiff_t offset) {index -= offset; return *this;}
    iterator operator+(std::ptrdiff_t offset) {iterator result(*this); return result += offset;}
    iterator operator-(std::ptrdiff_t offset) {iterator result(*this); return result -= offset;}
  };

  /** A class for constant iterators with its typical interface. */
  class const_iterator : public std::iterator<std::forward_iterator_tag, T>
  {
  private:
    const RingBuffer<T, n>& buffer; /**< The buffer. */
    std::size_t index; /**< Index of the current entry. */

  public:
    const_iterator(const RingBuffer<T, n>& buffer, std::size_t index) : buffer(buffer), index(index) {}
    const_iterator(const const_iterator& other) : const_iterator(other.buffer, other.index) {}
    const_iterator& operator=(const const_iterator& other) {return *new(this) const_iterator(other.buffer, other.index);}
    const T* operator->() const {return &buffer[index];}
    const T& operator*() const {return buffer[index];}
    bool operator==(const const_iterator& other) const {return index == other.index && &buffer == &other.buffer;}
    bool operator!=(const const_iterator& other) const {return index != other.index || &buffer != &other.buffer;}
    const_iterator& operator++() {++index; return *this;}
    const_iterator operator++(int) {iterator result(*this); ++index; return result;}
    const_iterator& operator+=(std::ptrdiff_t offset) {index += offset; return *this;}
    const_iterator& operator-=(std::ptrdiff_t offset) {index -= offset; return *this;}
    const_iterator operator+(std::ptrdiff_t offset) {iterator result(*this); return result += offset;}
    const_iterator operator-(std::ptrdiff_t offset) {iterator result(*this); return result -= offset;}
  };

  /**
   * Constructor.
   * @param capacity The maximum number of entries the buffer can store. If not specified,
   *                 the second template parameter is used as default capacity.
   */
  RingBuffer(std::size_t capacity = n) :
    buffer(reinterpret_cast<T*>(Memory::alignedMalloc(capacity * sizeof(T)))),
    allocated(capacity)
  {}

  /**
   * Copy constructor.
   * @param other The buffer this one is constructed from.
   */
  RingBuffer(const RingBuffer& other) :
    buffer(reinterpret_cast<T*>(Memory::alignedMalloc(other.allocated * sizeof(T)))),
    allocated(other.allocated)
  {
    for(std::size_t i = other.entries; i-- > 0;)
      push_front(other[i]);
  }

  /** Destructor. */
  ~RingBuffer()
  {
    clear();
    if(buffer)
      Memory::alignedFree(reinterpret_cast<char*>(buffer));
  }

  /**
   * Assignment operator.
   * @param other The buffer that is assigned to this one.
   */
  RingBuffer& operator=(const RingBuffer& other)
  {
    clear();
    if(allocated != other.allocated)
    {
      if(buffer)
        Memory::alignedFree(reinterpret_cast<char*>(buffer));
      buffer = reinterpret_cast<T*>(Memory::alignedMalloc(other.allocated * sizeof(T)));
    }
    allocated = other.allocated;
    head = 0;
    entries = 0;
    for(std::size_t i = other.entries; i-- > 0;)
      push_front(other[i]);
    return *this;
  }

  /** Empties the buffer. */
  void clear()
  {
    while(!empty())
      pop_back();
  }

  /**
   * Adds a new entry to the front of the buffer. The new entry is accessible under
   * index 0, front(), and *begin(). If the buffer was already full, the entry at
   * back() is lost.
   * @param value The value that is added to the buffer.
   */
  void push_front(const T& value)
  {
    ASSERT(allocated);
    if (entries < allocated)
    {
      new (buffer + head) T(value);
      ++entries;
    }
    else
      buffer[head] = value;
    head = (head + 1) % allocated;
  }

  /// @brief insert a new default constructed element at the front
  void push_front()
  {
    ASSERT(allocated);
    new (buffer + head) T();
    if (entries < allocated)
      ++entries;

    head = (head + 1) % allocated;
  }

  /**
   * Adds a new entry to the back of the buffer. The new entry is accessible under
   * index size()-1, back(), and *end(). If the buffer was already full, the entry at
   * front() is lost.
   * @param value The value that is added to the buffer.
   */
  void push_back(const T& value)
  {
    ASSERT(allocated);
    if (entries < allocated)
    {
      ++entries;
      new (buffer + ((allocated + head - entries) % allocated)) T(value);
    }
    else
    {
      head = (allocated + head - 1) % allocated; // overwrite old head
      buffer[(allocated + head - entries) % allocated] = value;
    }
  }

  /// @brief insert a new default constructed element at the back
  void push_back()
  {
    ASSERT(allocated);
    if (entries < allocated)
      ++entries;
    else
      head = (allocated + head - 1) % allocated; // overwrite old head

    new (buffer + ((allocated + head - entries) % allocated)) T();
  }



  /** Removes the entry back() from the buffer. */
  void pop_back()
  {
    ASSERT(!empty());
    buffer[(allocated + head - entries) % allocated].~T();
    --entries;
  }

  /** Removes the entry front() from the buffer. */
  void pop_front()
  {
    ASSERT(!empty());
    const std::size_t frontIndex = (allocated + head - 1) % allocated;
    buffer[frontIndex].~T();
    head = frontIndex;
    --entries;
  }


  /**
   * Access to the individual entries of the buffer.
   * @param index The index of the element. Element 0 is the same as front(), element
   *              size()-1 is the same as back().
   */
  T& operator[](std::size_t index) {ASSERT(!empty()); return buffer[(allocated + head - index - 1) % allocated];}
  const T& operator[](std::size_t index) const {ASSERT(!empty()); return buffer[(allocated + head - index - 1) % allocated];}

  T& from_back(std::size_t index) {ASSERT(!empty()); return buffer[(allocated + head - entries + index) % allocated];}
  const T& from_back(std::size_t index) const {ASSERT(!empty()); return buffer[(allocated + head - entries + index) % allocated];}


  /** Access the first element of the buffer. */
  T& front() {ASSERT(!empty()); return (*this)[0];}
  const T& front() const {ASSERT(!empty()); return (*this)[0];}

  /** Access the last element of the buffer. */
  T& back() {ASSERT(!empty()); return (*this)[entries - 1];}
  const T& back() const {ASSERT(!empty()); return (*this)[entries - 1];}

  /** The number of elements currently stored in the buffer. */
  std::size_t size() const {return entries;}

  /** The maximum number of elements that can be stored in the buffer. */
  std::size_t capacity() const {return allocated;}

  /**
   * Changes the capacity of the buffer. If it actually changes, the complexity is O(size()).
   * @param capacity The maximum number of entries the buffer can store.
   */
  void reserve(std::size_t capacity)
  {
    if(capacity != allocated)
    {
      while(size() > capacity)
        pop_back();

      T* prev = buffer;
      buffer = reinterpret_cast<T*>(Memory::alignedMalloc(capacity * sizeof(T)));
      if(head >= entries)
        std::memcpy(buffer, prev + head - entries, entries * sizeof(T));
      else
      {
        std::memcpy(buffer, prev + allocated - (entries - head), (entries - head) * sizeof(T));
        std::memcpy(buffer + entries - head, prev, head * sizeof(T));
      }
      allocated = capacity;
      head = allocated > entries ? entries : 0;
      if(prev)
        Memory::alignedFree(reinterpret_cast<char*>(prev));
    }
  }

  /** Is the buffer empty? */
  bool empty() const {return entries == 0;}

  /** Is the buffer full, i.e. will the next push_front() drop the element back()? */
  bool full() const {return entries == allocated;}

  /** Returns an iterator pointing at the front() of the buffer. */
  iterator begin() {return iterator(*this, 0);}
  const_iterator begin() const {return const_iterator(*this, 0);}

  /** Returns an iterator pointing at behind the back() of the buffer. */
  iterator end() {return iterator(*this, entries);}
  const_iterator end() const {return const_iterator(*this, entries);}

  /** return a string suitable for debugging changes to the ringbuffer */
  std::string debugString() const
  {
    const std::size_t frontIndex = (allocated + head - 0 - 1) % allocated;
    const std::size_t backIndex = (allocated + head - (entries-1) - 1) % allocated;

    return fmt::format("capacity {}, head {}, size/entries {}, front {}, back {}", 
                       allocated, head, entries, frontIndex, backIndex);
  }

protected:
  /** Is the buffer at the begin of a cycle? */
  bool cycled() const {return head == 0;}

  /** Is the current content wrapped around the end of the buffer? */
  bool wrapped() const {return head < entries;}
};
