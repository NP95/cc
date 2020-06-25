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

#ifndef CC_CACHE_H
#define CC_CACHE_H

#include "common.h"
#include <vector>

namespace cc {

struct CacheModelConfig {

  // The number of sets
  std::uint16_t sets_n = 1024;

  // The number of ways per set (degree of cache associativity).
  std::uint8_t ways_n = 4;

  // The length of a line in bytes.
  std::uint8_t line_bytes_n = 64;

  // The total number of cache lines.
  std::size_t lines() const;

  // The total cache capacity in bytes.
  std::size_t bytes() const;
};

struct CacheStatistics {
  // The total number of evictions experienced by the cache.
  std::uint64_t evictions = 0;

  // The total number of hits experienced by the cache.
  std::uint64_t hits = 0;

  // The total number of misses experienced by the cache.
  std::uint64_t misses = 0;
};

struct CacheAddressHelper {
  CacheAddressHelper(const CacheModelConfig& config);

  std::size_t offset_bits() const { return offset_bits_; }
  std::size_t line_bits() const { return line_bits_; }

  addr_t offset(const addr_t& a) const;
  addr_t line(const addr_t& a) const;
  addr_t tag(const addr_t& a) const;
  
 private:
  std::size_t offset_bits_;
  std::size_t line_bits_;
  CacheModelConfig config_;
};

template<typename T>
class CacheModel {
  
  using tag_type = addr_t;
  using line_id_type = addr_t;

  // Underlying type denoting each entry in the generic cache
  // structure.
  struct Line {
    bool valid = false;
    tag_type tag;
    T t;
  };
  
  using cache_type = typename std::vector<Line>;
  using line_iterator = typename std::vector<Line>::iterator;
  using const_line_iterator =
      typename std::vector<Line>::const_iterator;

 public:

  class Set {
    friend class CacheModel;

    Set(line_iterator begin, line_iterator end)
        : begin_(begin), end_(end) {}
   public:

    // Iterator to the 'begin' line (the zeroth line in the set).
    line_iterator begin() { return begin_; }
    const_line_iterator begin() const { return begin_; }

    // Iterator to the 'end' line (the line past the final line in the
    // set).
    line_iterator end() { return end_; }
    const_line_iterator end() const { return end_; }

    // Return true if the current tag can be installed in the cache
    // without the necessity of a prior eviction operation.
    bool requires_eviction(const tag_type& tag) const {
      for (const_line_iterator it = begin(); it != end(); ++it) {
        // Eviction whenever entry is not already present in the cache
        // and all ways in the set are already occupied.
        if (!it->valid || (it->tag == tag)) return false;
      }
      return true;
    }

    // Evict line pointer at by 'it' from the Set and by consequence
    // the owning cache.
    bool evict(line_iterator it) {
      it->valid = false;
    }

    // Find the line associated with the current tag otherwise return
    // the end iterator if not present in the cache.
    line_iterator find(const tag_type& tag) {
      for (line_iterator it = begin(); it != end(); ++it) {
        if (it->valid && (it->tag == tag)) return it;
      }
      return end();
    }

    // Find the line associated with the current tag otherwise return
    // the constant end iterator if not present in the cache.
    const_line_iterator find(const tag_type& tag) const {
      for (const_line_iterator it = begin(); it != end(); ++it) {
        if (it->valid && (it->tag == tag)) return it;
      }
      return end();
    }

    // Return true if line corresponding the current tag is present in
    // the cache.
    bool hit(const tag_type& tag) { return find(tag) != end(); }

   private:
    line_iterator begin_;
    line_iterator end_;
  };

  class ConstSet {
    friend class CacheModel;

    ConstSet(const_line_iterator begin, const_line_iterator end)
        : begin_(begin), end_(end) {}
   public:

    //
    const_line_iterator begin() { return begin_; }
    const_line_iterator begin() const { return begin_; }

    //
    const_line_iterator end() { return end_; }
    const_line_iterator end() const { return end_; }

    const_line_iterator find(const tag_type& tag) const {
      for (const_line_iterator it = begin(); it != end(); ++it) {
        if (it->valid && (it->tag == tag)) return it;
      }
      return end();
    }

    bool hit(const tag_type& tag) { return find(tag) != end(); }

   private:
    const_line_iterator begin_;
    const_line_iterator end_;
  };

 public:
  
  CacheModel(const CacheModelConfig& config) : config_(config), ah_(config) {
    cache_.resize(config.lines());
  }

  // Address Helper for the current cache configuration.
  const CacheAddressHelper& ah() const { return ah_; }
  CacheStatistics& stats() { return stats_; }
  const CacheStatistics& stats() const { return stats_; }

  bool hit(const addr_t& addr) const {
    return set(ah_.line(addr)).hit(ah_.tag(addr));
  }

  Set set(const line_id_type& line_id) {
    const std::size_t line_id_offset = (line_id * config_.ways_n);
    line_iterator begin = cache_.begin() + line_id_offset;
    return Set{begin, begin + config_.ways_n};
  }

  ConstSet set(const line_id_type& line_id) const {
    const std::size_t line_id_offset = (line_id * config_.ways_n);
    const_line_iterator begin = cache_.begin() + line_id_offset;
    return ConstSet{begin, begin + config_.ways_n};
  }

 private:
  CacheModelConfig config_;
  CacheAddressHelper ah_;
  CacheStatistics stats_;
  cache_type cache_;
};


} // namespace cc

#endif
