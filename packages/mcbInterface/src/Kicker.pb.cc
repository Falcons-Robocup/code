// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Kicker.proto

#include "Kicker.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

namespace kicker {
class EmptyDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Empty> _instance;
} _Empty_default_instance_;
class ShootPowerDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ShootPower> _instance;
} _ShootPower_default_instance_;
class HeightDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Height> _instance;
} _Height_default_instance_;
}  // namespace kicker
static void InitDefaultsEmpty_Kicker_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::kicker::_Empty_default_instance_;
    new (ptr) ::kicker::Empty();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::kicker::Empty::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_Empty_Kicker_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsEmpty_Kicker_2eproto}, {}};

static void InitDefaultsShootPower_Kicker_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::kicker::_ShootPower_default_instance_;
    new (ptr) ::kicker::ShootPower();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::kicker::ShootPower::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_ShootPower_Kicker_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsShootPower_Kicker_2eproto}, {}};

static void InitDefaultsHeight_Kicker_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::kicker::_Height_default_instance_;
    new (ptr) ::kicker::Height();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::kicker::Height::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_Height_Kicker_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsHeight_Kicker_2eproto}, {}};

void InitDefaults_Kicker_2eproto() {
  ::google::protobuf::internal::InitSCC(&scc_info_Empty_Kicker_2eproto.base);
  ::google::protobuf::internal::InitSCC(&scc_info_ShootPower_Kicker_2eproto.base);
  ::google::protobuf::internal::InitSCC(&scc_info_Height_Kicker_2eproto.base);
}

::google::protobuf::Metadata file_level_metadata_Kicker_2eproto[3];
constexpr ::google::protobuf::EnumDescriptor const** file_level_enum_descriptors_Kicker_2eproto = nullptr;
constexpr ::google::protobuf::ServiceDescriptor const** file_level_service_descriptors_Kicker_2eproto = nullptr;

const ::google::protobuf::uint32 TableStruct_Kicker_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::kicker::Empty, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::kicker::ShootPower, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::kicker::ShootPower, value_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::kicker::Height, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::kicker::Height, value_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::kicker::Empty)},
  { 5, -1, sizeof(::kicker::ShootPower)},
  { 11, -1, sizeof(::kicker::Height)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::kicker::_Empty_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::kicker::_ShootPower_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::kicker::_Height_default_instance_),
};

::google::protobuf::internal::AssignDescriptorsTable assign_descriptors_table_Kicker_2eproto = {
  {}, AddDescriptors_Kicker_2eproto, "Kicker.proto", schemas,
  file_default_instances, TableStruct_Kicker_2eproto::offsets,
  file_level_metadata_Kicker_2eproto, 3, file_level_enum_descriptors_Kicker_2eproto, file_level_service_descriptors_Kicker_2eproto,
};

const char descriptor_table_protodef_Kicker_2eproto[] =
  "\n\014Kicker.proto\022\006kicker\"\007\n\005Empty\"\033\n\nShoot"
  "Power\022\r\n\005value\030\001 \001(\002\"\027\n\006Height\022\r\n\005value\030"
  "\001 \001(\0022e\n\006Kicker\022,\n\005shoot\022\022.kicker.ShootP"
  "ower\032\r.kicker.Empty\"\000\022-\n\nset_height\022\016.ki"
  "cker.Height\032\r.kicker.Empty\"\000b\006proto3"
  ;
::google::protobuf::internal::DescriptorTable descriptor_table_Kicker_2eproto = {
  false, InitDefaults_Kicker_2eproto,
  descriptor_table_protodef_Kicker_2eproto,
  "Kicker.proto", &assign_descriptors_table_Kicker_2eproto, 196,
};

void AddDescriptors_Kicker_2eproto() {
  static constexpr ::google::protobuf::internal::InitFunc deps[1] =
  {
  };
 ::google::protobuf::internal::AddDescriptors(&descriptor_table_Kicker_2eproto, deps, 0);
}

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_Kicker_2eproto = []() { AddDescriptors_Kicker_2eproto(); return true; }();
namespace kicker {

// ===================================================================

void Empty::InitAsDefaultInstance() {
}
class Empty::HasBitSetters {
 public:
};

#if !defined(_MSC_VER) || _MSC_VER >= 1900
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Empty::Empty()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:kicker.Empty)
}
Empty::Empty(const Empty& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:kicker.Empty)
}

void Empty::SharedCtor() {
}

Empty::~Empty() {
  // @@protoc_insertion_point(destructor:kicker.Empty)
  SharedDtor();
}

void Empty::SharedDtor() {
}

void Empty::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Empty& Empty::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_Empty_Kicker_2eproto.base);
  return *internal_default_instance();
}


void Empty::Clear() {
// @@protoc_insertion_point(message_clear_start:kicker.Empty)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* Empty::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<Empty*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      default: {
        if ((tag & 7) == 4 || tag == 0) {
          ctx->EndGroup(tag);
          return ptr;
        }
        auto res = UnknownFieldParse(tag, {_InternalParse, msg},
          ptr, end, msg->_internal_metadata_.mutable_unknown_fields(), ctx);
        ptr = res.first;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr != nullptr);
        if (res.second) return ptr;
      }
    }  // switch
  }  // while
  return ptr;
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool Empty::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:kicker.Empty)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
  handle_unusual:
    if (tag == 0) {
      goto success;
    }
    DO_(::google::protobuf::internal::WireFormat::SkipField(
          input, tag, _internal_metadata_.mutable_unknown_fields()));
  }
success:
  // @@protoc_insertion_point(parse_success:kicker.Empty)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:kicker.Empty)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void Empty::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:kicker.Empty)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:kicker.Empty)
}

::google::protobuf::uint8* Empty::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:kicker.Empty)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:kicker.Empty)
  return target;
}

size_t Empty::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:kicker.Empty)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Empty::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:kicker.Empty)
  GOOGLE_DCHECK_NE(&from, this);
  const Empty* source =
      ::google::protobuf::DynamicCastToGenerated<Empty>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:kicker.Empty)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:kicker.Empty)
    MergeFrom(*source);
  }
}

void Empty::MergeFrom(const Empty& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:kicker.Empty)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

}

void Empty::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:kicker.Empty)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Empty::CopyFrom(const Empty& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:kicker.Empty)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Empty::IsInitialized() const {
  return true;
}

void Empty::Swap(Empty* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Empty::InternalSwap(Empty* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Empty::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_Kicker_2eproto);
  return ::file_level_metadata_Kicker_2eproto[kIndexInFileMessages];
}


// ===================================================================

void ShootPower::InitAsDefaultInstance() {
}
class ShootPower::HasBitSetters {
 public:
};

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ShootPower::kValueFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ShootPower::ShootPower()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:kicker.ShootPower)
}
ShootPower::ShootPower(const ShootPower& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  value_ = from.value_;
  // @@protoc_insertion_point(copy_constructor:kicker.ShootPower)
}

void ShootPower::SharedCtor() {
  value_ = 0;
}

ShootPower::~ShootPower() {
  // @@protoc_insertion_point(destructor:kicker.ShootPower)
  SharedDtor();
}

void ShootPower::SharedDtor() {
}

void ShootPower::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ShootPower& ShootPower::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_ShootPower_Kicker_2eproto.base);
  return *internal_default_instance();
}


void ShootPower::Clear() {
// @@protoc_insertion_point(message_clear_start:kicker.ShootPower)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  value_ = 0;
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* ShootPower::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<ShootPower*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // float value = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) != 13) goto handle_unusual;
        msg->set_value(::google::protobuf::io::UnalignedLoad<float>(ptr));
        ptr += sizeof(float);
        break;
      }
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->EndGroup(tag);
          return ptr;
        }
        auto res = UnknownFieldParse(tag, {_InternalParse, msg},
          ptr, end, msg->_internal_metadata_.mutable_unknown_fields(), ctx);
        ptr = res.first;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr != nullptr);
        if (res.second) return ptr;
      }
    }  // switch
  }  // while
  return ptr;
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool ShootPower::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:kicker.ShootPower)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // float value = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (13 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &value_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:kicker.ShootPower)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:kicker.ShootPower)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void ShootPower::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:kicker.ShootPower)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float value = 1;
  if (this->value() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->value(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:kicker.ShootPower)
}

::google::protobuf::uint8* ShootPower::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:kicker.ShootPower)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float value = 1;
  if (this->value() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->value(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:kicker.ShootPower)
  return target;
}

size_t ShootPower::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:kicker.ShootPower)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // float value = 1;
  if (this->value() != 0) {
    total_size += 1 + 4;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ShootPower::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:kicker.ShootPower)
  GOOGLE_DCHECK_NE(&from, this);
  const ShootPower* source =
      ::google::protobuf::DynamicCastToGenerated<ShootPower>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:kicker.ShootPower)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:kicker.ShootPower)
    MergeFrom(*source);
  }
}

void ShootPower::MergeFrom(const ShootPower& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:kicker.ShootPower)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.value() != 0) {
    set_value(from.value());
  }
}

void ShootPower::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:kicker.ShootPower)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ShootPower::CopyFrom(const ShootPower& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:kicker.ShootPower)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ShootPower::IsInitialized() const {
  return true;
}

void ShootPower::Swap(ShootPower* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ShootPower::InternalSwap(ShootPower* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(value_, other->value_);
}

::google::protobuf::Metadata ShootPower::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_Kicker_2eproto);
  return ::file_level_metadata_Kicker_2eproto[kIndexInFileMessages];
}


// ===================================================================

void Height::InitAsDefaultInstance() {
}
class Height::HasBitSetters {
 public:
};

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Height::kValueFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Height::Height()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:kicker.Height)
}
Height::Height(const Height& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  value_ = from.value_;
  // @@protoc_insertion_point(copy_constructor:kicker.Height)
}

void Height::SharedCtor() {
  value_ = 0;
}

Height::~Height() {
  // @@protoc_insertion_point(destructor:kicker.Height)
  SharedDtor();
}

void Height::SharedDtor() {
}

void Height::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Height& Height::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_Height_Kicker_2eproto.base);
  return *internal_default_instance();
}


void Height::Clear() {
// @@protoc_insertion_point(message_clear_start:kicker.Height)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  value_ = 0;
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* Height::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<Height*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // float value = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) != 13) goto handle_unusual;
        msg->set_value(::google::protobuf::io::UnalignedLoad<float>(ptr));
        ptr += sizeof(float);
        break;
      }
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->EndGroup(tag);
          return ptr;
        }
        auto res = UnknownFieldParse(tag, {_InternalParse, msg},
          ptr, end, msg->_internal_metadata_.mutable_unknown_fields(), ctx);
        ptr = res.first;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr != nullptr);
        if (res.second) return ptr;
      }
    }  // switch
  }  // while
  return ptr;
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool Height::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:kicker.Height)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // float value = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (13 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &value_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:kicker.Height)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:kicker.Height)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void Height::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:kicker.Height)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float value = 1;
  if (this->value() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->value(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:kicker.Height)
}

::google::protobuf::uint8* Height::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:kicker.Height)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float value = 1;
  if (this->value() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->value(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:kicker.Height)
  return target;
}

size_t Height::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:kicker.Height)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // float value = 1;
  if (this->value() != 0) {
    total_size += 1 + 4;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Height::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:kicker.Height)
  GOOGLE_DCHECK_NE(&from, this);
  const Height* source =
      ::google::protobuf::DynamicCastToGenerated<Height>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:kicker.Height)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:kicker.Height)
    MergeFrom(*source);
  }
}

void Height::MergeFrom(const Height& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:kicker.Height)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.value() != 0) {
    set_value(from.value());
  }
}

void Height::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:kicker.Height)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Height::CopyFrom(const Height& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:kicker.Height)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Height::IsInitialized() const {
  return true;
}

void Height::Swap(Height* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Height::InternalSwap(Height* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(value_, other->value_);
}

::google::protobuf::Metadata Height::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_Kicker_2eproto);
  return ::file_level_metadata_Kicker_2eproto[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace kicker
namespace google {
namespace protobuf {
template<> PROTOBUF_NOINLINE ::kicker::Empty* Arena::CreateMaybeMessage< ::kicker::Empty >(Arena* arena) {
  return Arena::CreateInternal< ::kicker::Empty >(arena);
}
template<> PROTOBUF_NOINLINE ::kicker::ShootPower* Arena::CreateMaybeMessage< ::kicker::ShootPower >(Arena* arena) {
  return Arena::CreateInternal< ::kicker::ShootPower >(arena);
}
template<> PROTOBUF_NOINLINE ::kicker::Height* Arena::CreateMaybeMessage< ::kicker::Height >(Arena* arena) {
  return Arena::CreateInternal< ::kicker::Height >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>