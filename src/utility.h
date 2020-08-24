//========================================================================== //
// Copyright (c) 2020, Stephen Henry
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//========================================================================== //

#ifndef CC_INCLUDE_CC_COMMON_H
#define CC_INCLUDE_CC_COMMON_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// Utility macro to indicate beginning of macro block.
#define MACRO_BEGIN do {

// Utility macro to indicate end of macro block.
#define MACRO_END } while(false)

namespace cc {

using addr_t = std::uint64_t;

template <typename T>
T log2ceil(T t) {
  int n = 0;
  while (t) {
    t >>= 1;
    n++;
  }
  return n;
}

template <typename T>
T mask(std::size_t bits) {
  if (bits == 0) return 0;

  T t = 1;
  return (t << bits) - 1;
}

//
//
class ArrayRenderer {
 public:
  ArrayRenderer() = default;
  std::string to_string() const;
  void add_item(const std::string& item);

 private:
  std::vector<std::string> items_;
};

//
//
class KVListRenderer {
  using kv_type = std::pair<std::string, std::string>;

 public:
  KVListRenderer() = default;

  //
  std::string to_string() const;

  //
  void add_field(const std::string& key, const std::string& value);

 private:
  // Key/Value pairs
  std::vector<kv_type> kvs_;
};

template <typename FwdIt>
typename std::iterator_traits<FwdIt>::value_type join(FwdIt begin, FwdIt end,
                                                      const char* sep = ".") {
  typename std::iterator_traits<FwdIt>::value_type v;
  while (begin != end) {
    v += *begin;
    if (++begin != end) v += sep;
  }
  return v;
}

template <typename IT>
void split(IT it, const std::string& name, const char* sep = ".") {
  std::string::size_type i = 0, j = 0;
  do {
    j = name.find(sep, i);
    if (j == std::string::npos) {
      *it++ = name.substr(i);
    } else {
      *it++ = name.substr(i, j - i);
      i = j + 1;
    }
  } while (j != std::string::npos);
}

// Convert some '.' separated path and replace with '_'
std::string flatten_path(const std::string& path);

// Join two strings to form path: a.b.c and d becomes a.b.c.d
std::string join_path(const std::string& base, const std::string& item);

template <typename>
class Pool;

// PooledItem is a Facade over a poolable base type T. Facade override
// base-types release method such that when called, the object is
// returned to a Pool instead of being deallocated.
//
template <typename T>
class PooledItem : public T {
  friend class Pool<T>;
  virtual ~PooledItem() = default;
  PooledItem() = default;

 public:
  virtual void release() const override { Pool<T>::release(this); }
};

//
//
template <typename T>
class Pool {
  // Utility class which encapsulates the notion of a free set of
  // PooledItems which is appropriately wound down upon destruction.
  class FreeSet {
   public:
    FreeSet() = default;

    ~FreeSet() {
      for (PooledItem<T>* item : items_) {
        delete item;
      }
    }

    bool empty() const { return items_.empty(); }
    PooledItem<T>* back() const { return items_.back(); }

    void push_back(PooledItem<T>* item) { items_.push_back(item); }
    void pop_back() { items_.pop_back(); }

   private:
    std::vector<PooledItem<T>*> items_;
  };

 public:
  // Construct new item from pool.
  static PooledItem<T>* construct() {
    check_free_set();
    PooledItem<T>* t = nullptr;
    if (!fs_->empty()) {
      // Review for UB: Pool contains old items. We wish to recover
      // the memory without performing the memory allocation more than
      // once. Therefore, invoke the destructor on the old item and
      // then invoke placement new over the old item.
      t = fs_->back();
      t->~T();
      t = new (t) PooledItem<T>{};
      fs_->pop_back();
    } else {
      t = new PooledItem<T>{};
    }
    return t;
  }

  // Release item back to pool.
  static void release(const PooledItem<T>* t) {
    check_free_set();
    // Magic, T is usually a const Message (by convention) but we wish
    // to retain this is in an non-constant form within the pool such
    // tha ancilliary may be added to it as required.
    PooledItem<T>* ut = const_cast<PooledItem<T>*>(t);
    fs_->push_back(ut);
  }

 private:
  static void check_free_set() {
    // Construct FreeSet if not already built.
    if (fs_) return;
    fs_ = std::make_unique<FreeSet>();
  }

  static inline std::unique_ptr<FreeSet> fs_;
};

//
//
struct Hexer {
  explicit Hexer() = default;

  // Currently rendering uppercase.
  bool upper_case() const { return upper_case_; }

  // Currently rendering with prefix.
  bool prefix() const { return prefix_; }

  // Convert some multi-byte value to hex equivalent.
  std::string to_hex(std::uint64_t x, std::size_t bits = 64) const;
  std::string to_hex(const char* c, std::size_t bits) const;

  // Convert integer in range [0, 16) to hex character.
  char hex_char(char x) const;

 private:
  // Render test with uppercase hexadecimal characters.
  bool upper_case_ = false;

  // Render text with '0x' prefix
  bool prefix_ = true;

  // Render with stripping leading zeros
  bool truncate_ = true;
};

const char* to_string(bool b);

}  // namespace cc

#endif
