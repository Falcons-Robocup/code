// Copyright 2019 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Kicker.proto

#ifndef PROTOBUF_INCLUDED_Kicker_2eproto
#define PROTOBUF_INCLUDED_Kicker_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3007000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3007001 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_Kicker_2eproto

// Internal implementation detail -- do not use these members.
struct TableStruct_Kicker_2eproto {
  static const ::google::protobuf::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::ParseTable schema[3]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors_Kicker_2eproto();
namespace kicker {
class Empty;
class EmptyDefaultTypeInternal;
extern EmptyDefaultTypeInternal _Empty_default_instance_;
class Height;
class HeightDefaultTypeInternal;
extern HeightDefaultTypeInternal _Height_default_instance_;
class ShootPower;
class ShootPowerDefaultTypeInternal;
extern ShootPowerDefaultTypeInternal _ShootPower_default_instance_;
}  // namespace kicker
namespace google {
namespace protobuf {
template<> ::kicker::Empty* Arena::CreateMaybeMessage<::kicker::Empty>(Arena*);
template<> ::kicker::Height* Arena::CreateMaybeMessage<::kicker::Height>(Arena*);
template<> ::kicker::ShootPower* Arena::CreateMaybeMessage<::kicker::ShootPower>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace kicker {

// ===================================================================

class Empty :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:kicker.Empty) */ {
 public:
  Empty();
  virtual ~Empty();

  Empty(const Empty& from);

  inline Empty& operator=(const Empty& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Empty(Empty&& from) noexcept
    : Empty() {
    *this = ::std::move(from);
  }

  inline Empty& operator=(Empty&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const Empty& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Empty* internal_default_instance() {
    return reinterpret_cast<const Empty*>(
               &_Empty_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Empty* other);
  friend void swap(Empty& a, Empty& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Empty* New() const final {
    return CreateMaybeMessage<Empty>(nullptr);
  }

  Empty* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Empty>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Empty& from);
  void MergeFrom(const Empty& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Empty* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // @@protoc_insertion_point(class_scope:kicker.Empty)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_Kicker_2eproto;
};
// -------------------------------------------------------------------

class ShootPower :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:kicker.ShootPower) */ {
 public:
  ShootPower();
  virtual ~ShootPower();

  ShootPower(const ShootPower& from);

  inline ShootPower& operator=(const ShootPower& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ShootPower(ShootPower&& from) noexcept
    : ShootPower() {
    *this = ::std::move(from);
  }

  inline ShootPower& operator=(ShootPower&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const ShootPower& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ShootPower* internal_default_instance() {
    return reinterpret_cast<const ShootPower*>(
               &_ShootPower_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(ShootPower* other);
  friend void swap(ShootPower& a, ShootPower& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ShootPower* New() const final {
    return CreateMaybeMessage<ShootPower>(nullptr);
  }

  ShootPower* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<ShootPower>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const ShootPower& from);
  void MergeFrom(const ShootPower& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ShootPower* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // float value = 1;
  void clear_value();
  static const int kValueFieldNumber = 1;
  float value() const;
  void set_value(float value);

  // @@protoc_insertion_point(class_scope:kicker.ShootPower)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  float value_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_Kicker_2eproto;
};
// -------------------------------------------------------------------

class Height :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:kicker.Height) */ {
 public:
  Height();
  virtual ~Height();

  Height(const Height& from);

  inline Height& operator=(const Height& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Height(Height&& from) noexcept
    : Height() {
    *this = ::std::move(from);
  }

  inline Height& operator=(Height&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const Height& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Height* internal_default_instance() {
    return reinterpret_cast<const Height*>(
               &_Height_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(Height* other);
  friend void swap(Height& a, Height& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Height* New() const final {
    return CreateMaybeMessage<Height>(nullptr);
  }

  Height* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Height>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Height& from);
  void MergeFrom(const Height& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Height* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // float value = 1;
  void clear_value();
  static const int kValueFieldNumber = 1;
  float value() const;
  void set_value(float value);

  // @@protoc_insertion_point(class_scope:kicker.Height)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  float value_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_Kicker_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Empty

// -------------------------------------------------------------------

// ShootPower

// float value = 1;
inline void ShootPower::clear_value() {
  value_ = 0;
}
inline float ShootPower::value() const {
  // @@protoc_insertion_point(field_get:kicker.ShootPower.value)
  return value_;
}
inline void ShootPower::set_value(float value) {
  
  value_ = value;
  // @@protoc_insertion_point(field_set:kicker.ShootPower.value)
}

// -------------------------------------------------------------------

// Height

// float value = 1;
inline void Height::clear_value() {
  value_ = 0;
}
inline float Height::value() const {
  // @@protoc_insertion_point(field_get:kicker.Height.value)
  return value_;
}
inline void Height::set_value(float value) {
  
  value_ = value;
  // @@protoc_insertion_point(field_set:kicker.Height.value)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace kicker

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // PROTOBUF_INCLUDED_Kicker_2eproto
