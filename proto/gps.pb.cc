// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: gps.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "gps.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace gnss_driver {
namespace pb {

namespace {

const ::google::protobuf::Descriptor* Gps_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Gps_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_gps_2eproto() {
  protobuf_AddDesc_gps_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "gps.proto");
  GOOGLE_CHECK(file != NULL);
  Gps_descriptor_ = file->message_type(0);
  static const int Gps_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Gps, header_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Gps, localization_),
  };
  Gps_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Gps_descriptor_,
      Gps::default_instance_,
      Gps_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Gps, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Gps, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Gps));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_gps_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Gps_descriptor_, &Gps::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_gps_2eproto() {
  delete Gps::default_instance_;
  delete Gps_reflection_;
}

void protobuf_AddDesc_gps_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::gnss_driver::pb::protobuf_AddDesc_header_2eproto();
  ::gnss_driver::pb::protobuf_AddDesc_pose_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\tgps.proto\022\016gnss_driver.pb\032\014header.prot"
    "o\032\npose.proto\"Y\n\003Gps\022&\n\006header\030\001 \001(\0132\026.g"
    "nss_driver.pb.Header\022*\n\014localization\030\002 \001"
    "(\0132\024.gnss_driver.pb.Pose", 144);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "gps.proto", &protobuf_RegisterTypes);
  Gps::default_instance_ = new Gps();
  Gps::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_gps_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_gps_2eproto {
  StaticDescriptorInitializer_gps_2eproto() {
    protobuf_AddDesc_gps_2eproto();
  }
} static_descriptor_initializer_gps_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int Gps::kHeaderFieldNumber;
const int Gps::kLocalizationFieldNumber;
#endif  // !_MSC_VER

Gps::Gps()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:gnss_driver.pb.Gps)
}

void Gps::InitAsDefaultInstance() {
  header_ = const_cast< ::gnss_driver::pb::Header*>(&::gnss_driver::pb::Header::default_instance());
  localization_ = const_cast< ::gnss_driver::pb::Pose*>(&::gnss_driver::pb::Pose::default_instance());
}

Gps::Gps(const Gps& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:gnss_driver.pb.Gps)
}

void Gps::SharedCtor() {
  _cached_size_ = 0;
  header_ = NULL;
  localization_ = NULL;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Gps::~Gps() {
  // @@protoc_insertion_point(destructor:gnss_driver.pb.Gps)
  SharedDtor();
}

void Gps::SharedDtor() {
  if (this != default_instance_) {
    delete header_;
    delete localization_;
  }
}

void Gps::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Gps::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Gps_descriptor_;
}

const Gps& Gps::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_gps_2eproto();
  return *default_instance_;
}

Gps* Gps::default_instance_ = NULL;

Gps* Gps::New() const {
  return new Gps;
}

void Gps::Clear() {
  if (_has_bits_[0 / 32] & 3) {
    if (has_header()) {
      if (header_ != NULL) header_->::gnss_driver::pb::Header::Clear();
    }
    if (has_localization()) {
      if (localization_ != NULL) localization_->::gnss_driver::pb::Pose::Clear();
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Gps::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gnss_driver.pb.Gps)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .gnss_driver.pb.Header header = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(18)) goto parse_localization;
        break;
      }

      // optional .gnss_driver.pb.Pose localization = 2;
      case 2: {
        if (tag == 18) {
         parse_localization:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_localization()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:gnss_driver.pb.Gps)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gnss_driver.pb.Gps)
  return false;
#undef DO_
}

void Gps::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gnss_driver.pb.Gps)
  // optional .gnss_driver.pb.Header header = 1;
  if (has_header()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->header(), output);
  }

  // optional .gnss_driver.pb.Pose localization = 2;
  if (has_localization()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->localization(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gnss_driver.pb.Gps)
}

::google::protobuf::uint8* Gps::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:gnss_driver.pb.Gps)
  // optional .gnss_driver.pb.Header header = 1;
  if (has_header()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->header(), target);
  }

  // optional .gnss_driver.pb.Pose localization = 2;
  if (has_localization()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        2, this->localization(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gnss_driver.pb.Gps)
  return target;
}

int Gps::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional .gnss_driver.pb.Header header = 1;
    if (has_header()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->header());
    }

    // optional .gnss_driver.pb.Pose localization = 2;
    if (has_localization()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->localization());
    }

  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Gps::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Gps* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Gps*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Gps::MergeFrom(const Gps& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_header()) {
      mutable_header()->::gnss_driver::pb::Header::MergeFrom(from.header());
    }
    if (from.has_localization()) {
      mutable_localization()->::gnss_driver::pb::Pose::MergeFrom(from.localization());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Gps::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Gps::CopyFrom(const Gps& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Gps::IsInitialized() const {

  return true;
}

void Gps::Swap(Gps* other) {
  if (other != this) {
    std::swap(header_, other->header_);
    std::swap(localization_, other->localization_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Gps::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Gps_descriptor_;
  metadata.reflection = Gps_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace pb
}  // namespace gnss_driver

// @@protoc_insertion_point(global_scope)
