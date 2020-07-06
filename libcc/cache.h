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

#ifndef CC_LIBCC_CACHE_H
#define CC_LIBCC_CACHE_H

#include <tuple>
#include <vector>

#include "cc/cfgs.h"
#include "cc/types.h"

namespace cc {

//
//
struct CacheStatistics {
  // The total number of evictions experienced by the cache.
  std::uint64_t evictions = 0;

  // The total number of hits experienced by the cache.
  std::uint64_t hits = 0;

  // The total number of misses experienced by the cache.
  std::uint64_t misses = 0;
};

//
//
struct CacheAddressHelper {
  CacheAddressHelper(const CacheModelConfig& config);

  std::size_t offset_bits() const { return offset_bits_; }
  std::size_t line_bits() const { return line_bits_; }

  addr_t offset(const addr_t& a) const;
  addr_t set(const addr_t& a) const;
  addr_t tag(const addr_t& a) const;

 private:
  std::size_t offset_bits_;
  std::size_t line_bits_;
  CacheModelConfig config_;
};

//
//
template <typename T>
class CacheModel {
 public:
  class Set;
  using tag_type = addr_t;
  using line_id_type = addr_t;

  // Underlying type denoting each entry in the generic cache
  // structure.
  struct Line {
    friend class CacheModel;
    friend class CacheModel::Set;

    // Accessors;
    bool valid() const { return valid_; }
    tag_type tag() const { return tag_; }
    T& t() { return t_; }
    const T& t() const { return t_; }

   private:
    void valid(bool valid) { valid_ = valid; }
    void tag(tag_type tag) { tag_ = tag; }
    void t(const T& t) { t_ = t; }

    bool valid_ = false;
    tag_type tag_;
    T t_;
  };

  using cache_type = typename std::vector<Line>;

  using line_iterator = typename std::vector<Line>::iterator;
  using const_line_iterator = typename std::vector<Line>::const_iterator;

  // An interator denoting the location of a line within the class.
  //
  class LineIterator {
    friend class CacheModel;
    friend class Set;

    friend bool operator==(const LineIterator& lhs, const LineIterator& rhs) {
      return lhs.raw_ == rhs.raw_;
    }

    friend bool operator!=(const LineIterator& lhs, const LineIterator& rhs) {
      return !operator==(lhs.raw_, rhs.raw_);
    }

    LineIterator(CacheModel* cache, line_iterator raw)
        : cache_(cache), raw_(raw) {}

   public:
    LineIterator() : cache_(nullptr) {}

    Line& line() { return *raw_; }
    const Line& line() const { return *raw_; }
    CacheModel* cache() const { return cache_; }

    // Pre-/Post- Increment operators
    LineIterator& operator++() {
      ++raw_;
      return *this;
    }
    LineIterator operator++(int) const {
      return LineIterator(cache(), raw_ + 1);
    }

   private:
    line_iterator raw() const { return raw_; }

    CacheModel* cache_ = nullptr;
    line_iterator raw_;
  };

  // An constant iterator denoting the location of a line within the class.
  //
  class ConstLineIterator {
    friend class Set;
    friend class CacheModel;

    friend bool operator==(const ConstLineIterator& lhs,
                           const ConstLineIterator& rhs) {
      return lhs.raw_ == rhs.raw_;
    }

    friend bool operator!=(const ConstLineIterator& lhs,
                           const ConstLineIterator& rhs) {
      return !operator==(lhs.raw_, rhs.raw_);
    }

    ConstLineIterator(const CacheModel* cache, const_line_iterator raw)
        : cache_(cache), raw_(raw) {}
    ConstLineIterator(const LineIterator& it)
        : cache_(it.cache()), raw_(it.raw()) {}

   public:
    const Line& line() const { return *raw_; }
    const CacheModel* cache() const { return cache_; }

    // Pre-/Post- Increment operators
    ConstLineIterator& operator++() {
      ++raw_;
      return *this;
    }
    ConstLineIterator operator++(int) const {
      return ConstLineIterator(cache(), raw_ + 1);
    }

   private:
    const_line_iterator raw() const { return raw_; }

    const CacheModel* cache_ = nullptr;
    const_line_iterator raw_;
  };

  class Evictor {
  public:
    enum class Policy { First };
    
    explicit Evictor(Policy policy = Policy::First)
      : policy_(policy) {}

    Policy policy() const { return policy_; }

    std::pair<LineIterator, bool> nominate(
       LineIterator begin, LineIterator end) const {
      LineIterator it;
      // Firstly consider empty lines within the set.
      it = begin;
      while (it != end) {
	const Line& line = it.line();
	if (!line.valid()) return std::make_pair(it, false);
	++it;
      }
      // Otherwise, consider lines which can be evicted.
      it = begin;
      std::vector<LineIterator> ns;
      while (it != end) {
	const Line& line = it.line();
	if (line.t()->is_evictable()) ns.push_back(it);
      }
      // If no nominations selected, return end()
      if (ns.empty()) return std::make_pair(end, false);

      // Otherwise, select according to selected policy.
      switch (policy()) {
      case Policy::First: {
	return std::make_pair(*ns.begin(), false);
      } break;
      default: {
	return std::make_pair(end, false);
      } break;
      }
    }
  private:
    Policy policy_;
  };

  // Structure representing a collection of lines with the same set.
  //
  class Set {
    friend class CacheModel;

    Set(LineIterator begin, LineIterator end) : begin_(begin), end_(end) {}

   public:
    // Iterator to the 'begin' line (the zeroth line in the set).
    LineIterator begin() { return begin_; }
    ConstLineIterator begin() const { return begin_; }

    // Iterator to the 'end' line (the line past the final line in the
    // set).
    LineIterator end() { return end_; }
    ConstLineIterator end() const { return ConstLineIterator(end_); }

    // Return true if the current tag can be installed in the cache
    // without the necessity of a prior eviction operation.
    bool requires_eviction(const tag_type& tag) const {
      for (ConstLineIterator it = begin(); it != end(); ++it) {
        // Eviction whenever entry is not already present in the cache
        // and all ways in the set are already occupied.
        const Line& line = it.line();
        if (!line.valid() || (line.tag() == tag)) return false;
      }
      return true;
    }

    bool install(LineIterator it, addr_t tag, const T& t) {
      // Validate that the current location the set is invalid. An
      // entry cannot be installed in the cache unless the data which is
      // already present has been evicted.

      Line& line = it.line();
      if (line.valid()) return false;

      line.valid(true);
      line.tag(tag);
      line.t(t);
      return true;
    }

    bool update(LineIterator it, addr_t tag, const T& t) {
      // Validate that the current line entry corresponds to a valid
      // entry which points to the current address in memory. Unlike
      // 'install', this method expects that the entry in the cache is
      // already present and is simply being overwritten with new data.
      Line& line = it.line();
      if (!line.valid() || (line.valid() && line.tag() != tag)) return false;

      line.t(t);
      return true;
    }

    // Evict line pointer at by 'it' from the Set and by consequence
    // the owning cache.
    bool evict(LineIterator it) {
      Line& line = it.line();
      line.valid(false);
      return true;
    }

    // Find the line associated with the current tag otherwise return
    // the end iterator if not present in the cache.
    LineIterator find(const tag_type& tag) {
      for (LineIterator it = begin(); it != end(); ++it) {
        Line& line = it.line();
        if (line.valid() && (line.tag() == tag)) return it;
      }
      return end();
    }

    // Find the line associated with the current tag otherwise return
    // the constant end iterator if not present in the cache.
    ConstLineIterator find(const tag_type& tag) const {
      for (ConstLineIterator it = begin(); it != end(); ++it) {
        const Line& line = it.line();
        if (line.valid() && (line.tag() == tag)) return it;
      }
      return end();
    }

    // Return true if the line corresponding to the current tag is
    // present in the cache and additionally return the state
    // associated with the line.
    bool hit(const tag_type& tag, T& t) {
      LineIterator it = find(tag);
      if (it == end()) return false;

      Line& line = it.line();
      t = line.t();
      return true;
    }

    // Return true if line corresponding the current tag is present in
    // the cache.
    bool hit(const tag_type& tag) const { return find(tag) != end(); }

   private:
    LineIterator begin_;
    LineIterator end_;
  };

  // Constant (immutable) structure repreenting a collection of lines
  // within the same set of a parent (owning) cache.
  //
  class ConstSet {
    friend class CacheModel;

    ConstSet(ConstLineIterator begin, ConstLineIterator end)
        : begin_(begin), end_(end) {}

   public:
    //
    ConstLineIterator begin() { return begin_; }
    ConstLineIterator begin() const { return begin_; }

    //
    ConstLineIterator end() { return end_; }
    ConstLineIterator end() const { return end_; }

    ConstLineIterator find(const tag_type& tag) const {
      for (ConstLineIterator it = begin(); it != end(); ++it) {
        const Line& line = it.line();
        if (line.valid() && (line.tag() == tag)) return it;
      }
      return end();
    }

    bool hit(const tag_type& tag) { return find(tag) != end(); }

   private:
    ConstLineIterator begin_;
    ConstLineIterator end_;
  };

 public:
  CacheModel(const CacheModelConfig& config) : config_(config), ah_(config) {
    cache_.resize(config.lines());
  }

  // Address Helper for the current cache configuration.
  const CacheAddressHelper& ah() const { return ah_; }

  // Cache statistics record.
  CacheStatistics& stats() { return stats_; }
  const CacheStatistics& stats() const { return stats_; }

  // Return true if address is present in the cache.
  bool hit(const addr_t& addr) const {
    return set(ah_.set(addr)).hit(ah_.tag(addr));
  }

  // Return cache set corresponding to the line id.
  Set set(const line_id_type& line_id) {
    const std::size_t line_id_offset = (line_id * config_.ways_n);
    const LineIterator begin{this, cache_.begin() + line_id_offset};
    const LineIterator end{this,
                           cache_.begin() + line_id_offset + config_.ways_n};
    return Set{begin, end};
  }

  // Return cache set corresponding to the line id (constant).
  ConstSet set(const line_id_type& line_id) const {
    const std::size_t line_id_offset = (line_id * config_.ways_n);
    const ConstLineIterator begin{this, cache_.begin() + line_id_offset};
    const ConstLineIterator end{
        this, cache_.begin() + line_id_offset + config_.ways_n};
    return ConstSet{begin, end};
  }

  void invalidate() {
    for (Line& l : cache_) l.valid(false);
  }

 private:
  // Current cache configuration.
  CacheModelConfig config_;

  // Helper class to assist with address field extraction.
  CacheAddressHelper ah_;

  // Record maintain the set of cache statistics.
  CacheStatistics stats_;

  // Cache state structure.
  cache_type cache_;
};

}  // namespace cc

#endif
