#include "nt_node.h"
#include <cstring>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

NT4::NT4() { inst = nt::NetworkTableInstance::GetDefault(); }

NT4::~NT4() {
  // Intentionally empty to prevent closing the connection on scene reload
}

void NT4::start_client(String server_ip) {
  inst.StartClient4("Godot");
  inst.SetServer(server_ip.utf8().get_data());
  UtilityFunctions::print("NT4 Client connecting to " + server_ip);
}

// Helper for fast update rate (200Hz)
nt::PubSubOptions GetFastInterval() {
  nt::PubSubOptions opts;
  opts.periodic = 0.01; // 100Hz
  opts.sendAll = true;  // Don't miss any updates
  return opts;
}

double NT4::get_number(String topic_name, double default_val) {
  std::string key = topic_name.utf8().get_data();
  auto it = double_subs.find(key);
  if (it == double_subs.end()) {
    nt::DoubleSubscriber sub =
        inst.GetDoubleTopic(key).Subscribe(default_val, GetFastInterval());
    double_subs.emplace(key, std::move(sub));
    it = double_subs.find(key);
  }
  return it->second.Get();
}

#include <array>
#include <string_view>

void NT4::subscribe_to_all() {
  static constexpr std::array<std::string_view, 1> prefixes = {
      std::string_view("/")};
  all_sub =
      std::make_unique<nt::MultiSubscriber>(inst, prefixes, GetFastInterval());
}

Variant NT4::get_value(String topic_name, Variant default_val) {
  std::string key = topic_name.utf8().get_data();

  // Most robust: Check Topic Type.
  nt::Topic topic = inst.GetTopic(key);
  nt::TopicInfo info = topic.GetInfo();

  // If undefined, return default
  if (info.type == NT_UNASSIGNED)
    return default_val;

  switch (info.type) {
  case NT_BOOLEAN: {
    auto it = boolean_subs.find(key);
    if (it == boolean_subs.end()) {
      boolean_subs.emplace(
          key, inst.GetBooleanTopic(key).Subscribe(false, GetFastInterval()));
      it = boolean_subs.find(key);
    }
    return it->second.Get();
  }
  case NT_DOUBLE: {
    auto it = double_subs.find(key);
    if (it == double_subs.end()) {
      double_subs.emplace(
          key, inst.GetDoubleTopic(key).Subscribe(0.0, GetFastInterval()));
      it = double_subs.find(key);
    }
    return it->second.Get();
  }
  case NT_FLOAT: {
    auto it = float_subs.find(key);
    if (it == float_subs.end()) {
      float_subs.emplace(
          key, inst.GetFloatTopic(key).Subscribe(0.0f, GetFastInterval()));
      it = float_subs.find(key);
    }
    return it->second.Get();
  }
  case NT_INTEGER: {
    auto it = integer_subs.find(key);
    if (it == integer_subs.end()) {
      integer_subs.emplace(
          key, inst.GetIntegerTopic(key).Subscribe(0, GetFastInterval()));
      it = integer_subs.find(key);
    }
    return it->second.Get();
  }
  case NT_STRING: {
    auto it = string_subs.find(key);
    if (it == string_subs.end()) {
      string_subs.emplace(
          key, inst.GetStringTopic(key).Subscribe("", GetFastInterval()));
      it = string_subs.find(key);
    }
    return String(it->second.Get().c_str());
  }
  case NT_BOOLEAN_ARRAY: {
    auto it = boolean_array_subs.find(key);
    if (it == boolean_array_subs.end()) {
      boolean_array_subs.emplace(
          key, inst.GetBooleanArrayTopic(key).Subscribe({}, GetFastInterval()));
      it = boolean_array_subs.find(key);
    }
    auto res = it->second.Get();
    Array arr;
    for (int v : res)
      arr.append(v != 0);
    return arr;
  }
  case NT_DOUBLE_ARRAY: {
    auto it = double_array_subs.find(key);
    if (it == double_array_subs.end()) {
      double_array_subs.emplace(
          key, inst.GetDoubleArrayTopic(key).Subscribe({}, GetFastInterval()));
      it = double_array_subs.find(key);
    }
    auto res = it->second.Get();
    PackedFloat64Array arr;
    arr.resize(res.size());
    for (size_t i = 0; i < res.size(); i++)
      arr[i] = res[i];
    return arr;
  }
  case NT_FLOAT_ARRAY: {
    auto it = float_array_subs.find(key);
    if (it == float_array_subs.end()) {
      float_array_subs.emplace(
          key, inst.GetFloatArrayTopic(key).Subscribe({}, GetFastInterval()));
      it = float_array_subs.find(key);
    }
    auto res = it->second.Get();
    PackedFloat64Array arr;
    arr.resize(res.size());
    for (size_t i = 0; i < res.size(); i++)
      arr[i] = res[i];
    return arr;
  }
  case NT_STRING_ARRAY: {
    auto it = string_array_subs.find(key);
    if (it == string_array_subs.end()) {
      string_array_subs.emplace(
          key, inst.GetStringArrayTopic(key).Subscribe({}, GetFastInterval()));
      it = string_array_subs.find(key);
    }
    auto res = it->second.Get();
    PackedStringArray arr;
    arr.resize(res.size());
    for (size_t i = 0; i < res.size(); i++)
      arr[i] = String(res[i].c_str());
    return arr;
  }
  case NT_RAW: {
    auto it = raw_subs.find(key);
    if (it == raw_subs.end()) {
      // "struct" type string is common but not strictly required for generic
      // RAW, but we need *some* type string. "raw" is safe.
      nt::RawSubscriber sub =
          inst.GetRawTopic(key).Subscribe("raw", {}, GetFastInterval());
      raw_subs.emplace(key, std::move(sub));
      it = raw_subs.find(key);
    }
    std::vector<uint8_t> res = it->second.Get();
    PackedByteArray arr;
    arr.resize(res.size());
    for (size_t i = 0; i < res.size(); i++) {
      arr[i] = res[i];
    }
    return arr;
  }
  default:
    return default_val;
  }
}

void NT4::set_number(String topic_name, double value) {
  std::string key = topic_name.utf8().get_data();
  auto it = double_pubs.find(key);
  if (it == double_pubs.end()) {
    nt::DoublePublisher pub =
        inst.GetDoubleTopic(key).Publish(GetFastInterval());
    double_pubs.emplace(key, std::move(pub));
    it = double_pubs.find(key);
  }
  it->second.Set(value);
}

PackedFloat64Array NT4::get_number_array(String topic_name,
                                         PackedFloat64Array default_val) {
  std::string key = topic_name.utf8().get_data();
  auto it = double_array_subs.find(key);

  // We need a std::vector for the default value to pass to NT
  std::vector<double> nt_default;

  if (it == double_array_subs.end()) {
    // subscribe
    nt::DoubleArraySubscriber sub =
        inst.GetDoubleArrayTopic(key).Subscribe(nt_default, GetFastInterval());
    double_array_subs.emplace(key, std::move(sub));
    it = double_array_subs.find(key);
  }

  // Convert std::vector (NT) -> PackedFloat64Array (Godot)
  std::vector<double> result = it->second.Get();

  PackedFloat64Array godot_array;
  godot_array.resize(result.size());
  for (size_t i = 0; i < result.size(); i++) {
    godot_array[i] = result[i];
  }
  return godot_array;
}

void NT4::set_number_array(String topic_name, PackedFloat64Array value) {
  std::string key = topic_name.utf8().get_data();
  auto it = double_array_pubs.find(key);
  if (it == double_array_pubs.end()) {
    nt::DoubleArrayPublisher pub =
        inst.GetDoubleArrayTopic(key).Publish(GetFastInterval());
    double_array_pubs.emplace(key, std::move(pub));
    it = double_array_pubs.find(key);
  }

  std::vector<double> std_vec;
  std_vec.resize(value.size());
  for (int i = 0; i < value.size(); i++) {
    std_vec[i] = value[i];
  }
  it->second.Set(std_vec);
}

// --- Boolean ---
bool NT4::get_boolean(String topic_name, bool default_val) {
  std::string key = topic_name.utf8().get_data();
  auto it = boolean_subs.find(key);
  if (it == boolean_subs.end()) {
    nt::BooleanSubscriber sub =
        inst.GetBooleanTopic(key).Subscribe(default_val, GetFastInterval());
    boolean_subs.emplace(key, std::move(sub));
    it = boolean_subs.find(key);
  }
  return it->second.Get();
}

void NT4::set_boolean(String topic_name, bool value) {
  std::string key = topic_name.utf8().get_data();
  auto it = boolean_pubs.find(key);
  if (it == boolean_pubs.end()) {
    nt::BooleanPublisher pub =
        inst.GetBooleanTopic(key).Publish(GetFastInterval());
    boolean_pubs.emplace(key, std::move(pub));
    it = boolean_pubs.find(key);
  }
  it->second.Set(value);
}

// --- String ---
String NT4::get_string(String topic_name, String default_val) {
  std::string key = topic_name.utf8().get_data();
  auto it = string_subs.find(key);
  if (it == string_subs.end()) {
    nt::StringSubscriber sub = inst.GetStringTopic(key).Subscribe(
        default_val.utf8().get_data(), GetFastInterval());
    string_subs.emplace(key, std::move(sub));
    it = string_subs.find(key);
  }
  return String(it->second.Get().c_str());
}

void NT4::set_string(String topic_name, String value) {
  std::string key = topic_name.utf8().get_data();
  auto it = string_pubs.find(key);
  if (it == string_pubs.end()) {
    nt::StringPublisher pub =
        inst.GetStringTopic(key).Publish(GetFastInterval());
    string_pubs.emplace(key, std::move(pub));
    it = string_pubs.find(key);
  }
  it->second.Set(value.utf8().get_data());
}

// --- Boolean Array ---
Array NT4::get_boolean_array(String topic_name, Array default_val) {
  std::string key = topic_name.utf8().get_data();
  auto it = boolean_array_subs.find(key);

  if (it == boolean_array_subs.end()) {
    std::vector<int> nt_default;
    // We'd strictly need to convert default_val to vector<int> here but empty
    // is usually fine for initial sub
    nt::BooleanArraySubscriber sub =
        inst.GetBooleanArrayTopic(key).Subscribe(nt_default, GetFastInterval());
    boolean_array_subs.emplace(key, std::move(sub));
    it = boolean_array_subs.find(key);
  }

  std::vector<int> result = it->second.Get();
  Array godot_array;
  for (int val : result) {
    godot_array.append(val != 0);
  }
  return godot_array;
}

void NT4::set_boolean_array(String topic_name, Array value) {
  std::string key = topic_name.utf8().get_data();
  auto it = boolean_array_pubs.find(key);
  if (it == boolean_array_pubs.end()) {
    nt::BooleanArrayPublisher pub =
        inst.GetBooleanArrayTopic(key).Publish(GetFastInterval());
    boolean_array_pubs.emplace(key, std::move(pub));
    it = boolean_array_pubs.find(key);
  }

  std::vector<int> std_vec;
  std_vec.reserve(value.size());
  for (int i = 0; i < value.size(); i++) {
    std_vec.push_back(value[i] ? 1 : 0);
  }
  it->second.Set(std_vec);
}

// --- String Array ---
PackedStringArray NT4::get_string_array(String topic_name,
                                        PackedStringArray default_val) {
  std::string key = topic_name.utf8().get_data();
  auto it = string_array_subs.find(key);

  if (it == string_array_subs.end()) {
    std::vector<std::string> nt_default; // Empty default
    nt::StringArraySubscriber sub =
        inst.GetStringArrayTopic(key).Subscribe(nt_default, GetFastInterval());
    string_array_subs.emplace(key, std::move(sub));
    it = string_array_subs.find(key);
  }

  std::vector<std::string> result = it->second.Get();
  PackedStringArray godot_array;
  godot_array.resize(result.size());
  for (size_t i = 0; i < result.size(); i++) {
    godot_array[i] = String(result[i].c_str());
  }
  return godot_array;
}

void NT4::set_string_array(String topic_name, PackedStringArray value) {
  std::string key = topic_name.utf8().get_data();
  auto it = string_array_pubs.find(key);
  if (it == string_array_pubs.end()) {
    nt::StringArrayPublisher pub =
        inst.GetStringArrayTopic(key).Publish(GetFastInterval());
    string_array_pubs.emplace(key, std::move(pub));
    it = string_array_pubs.find(key);
  }

  std::vector<std::string> std_vec;
  std_vec.reserve(value.size());
  for (int i = 0; i < value.size(); i++) {
    std_vec.push_back(value[i].utf8().get_data());
  }
  it->second.Set(std_vec);
}

// --- Geometry Helpers ---
std::vector<uint8_t>
get_raw_bytes(nt::NetworkTableInstance &inst,
              std::map<std::string, nt::RawSubscriber> &subs, std::string key) {
  auto it = subs.find(key);
  if (it == subs.end()) {
    nt::RawSubscriber sub =
        inst.GetRawTopic(key).Subscribe("struct", {}, GetFastInterval());
    subs.emplace(key, std::move(sub));
    it = subs.find(key);
  }
  return it->second.Get();
}

Vector2 NT4::get_translation2d(String topic, Vector2 default_val) {
  std::string key = topic.utf8().get_data();
  std::vector<uint8_t> data = get_raw_bytes(inst, raw_subs, key);

  if (data.size() < 16)
    return default_val;

  double fx, fy; // FRC Coordinates
  std::memcpy(&fx, data.data(), 8);
  std::memcpy(&fy, data.data() + 8, 8);

  // MAPPING:
  // FRC X (Forward) -> Godot X (Right)
  // FRC Y (Left)    -> Godot -Y (Up on screen)
  return Vector2(fx, -fy);
}

double NT4::get_rotation2d(String topic, double default_val) {
  std::string key = topic.utf8().get_data();
  std::vector<uint8_t> data = get_raw_bytes(inst, raw_subs, key);

  // Rotation2d struct is just 1 double (8 bytes)
  if (data.size() < 8)
    return default_val;

  double rot;
  std::memcpy(&rot, data.data(), 8);

  // FRC is CCW, Godot is CW. Negate the angle.
  return -rot;
}

Transform2D NT4::get_pose2d(String topic, Transform2D default_val) {
  std::string key = topic.utf8().get_data();
  std::vector<uint8_t> data = get_raw_bytes(inst, raw_subs, key);

  if (data.size() < 24)
    return default_val;

  double fx, fy, frot;
  std::memcpy(&fx, data.data(), 8);
  std::memcpy(&fy, data.data() + 8, 8);
  std::memcpy(&frot, data.data() + 16, 8);

  // 1. Convert Rotation (Negate)
  double godot_rot = -frot;

  // 2. Convert Position (Flip Y)
  Vector2 godot_pos(fx, -fy);

  // Godot Transform2D constructor takes (Rotation, Position)
  return Transform2D(static_cast<real_t>(godot_rot), godot_pos);
}

Vector3 NT4::get_translation3d(String topic, Vector3 default_val) {
  std::string key = topic.utf8().get_data();
  std::vector<uint8_t> data = get_raw_bytes(inst, raw_subs, key);

  if (data.size() < 24)
    return default_val;

  double fx, fy, fz; // FRC Coordinates
  std::memcpy(&fx, data.data(), 8);
  std::memcpy(&fy, data.data() + 8, 8);
  std::memcpy(&fz, data.data() + 16, 8);

  // MAP: FRC(x, y, z) -> Godot(-y, z, -x)
  return Vector3(-fy, fz, -fx);
}

Quaternion NT4::get_rotation3d(String topic, Quaternion default_val) {
  std::string key = topic.utf8().get_data();
  std::vector<uint8_t> data = get_raw_bytes(inst, raw_subs, key);

  if (data.size() < 32)
    return default_val;

  double w, fx, fy, fz; // FRC Coordinates
  std::memcpy(&w, data.data(), 8);
  std::memcpy(&fx, data.data() + 8, 8);
  std::memcpy(&fy, data.data() + 16, 8);
  std::memcpy(&fz, data.data() + 24, 8);

  // MAP: FRC(x, y, z, w) -> Godot(-y, z, -x, w)
  return Quaternion(-fy, fz, -fx, w);
}

Transform3D NT4::get_pose3d(String topic, Transform3D default_val) {
  std::string key = topic.utf8().get_data();
  std::vector<uint8_t> data = get_raw_bytes(inst, raw_subs, key);

  if (data.size() < 56)
    return default_val;

  // 1. Unpack FRC Position
  double tx, ty, tz;
  std::memcpy(&tx, data.data(), 8);
  std::memcpy(&ty, data.data() + 8, 8);
  std::memcpy(&tz, data.data() + 16, 8);

  // 2. Unpack FRC Rotation
  double qw, qx, qy, qz;
  std::memcpy(&qw, data.data() + 24, 8);
  std::memcpy(&qx, data.data() + 32, 8);
  std::memcpy(&qy, data.data() + 40, 8);
  std::memcpy(&qz, data.data() + 48, 8);

  // 3. Convert to Godot Coordinates
  // Pos: (-y, z, -x)
  Vector3 origin(-ty, tz, -tx);

  // Rot: (-y, z, -x, w)
  Quaternion q(-qy, qz, -qx, qw);

  return Transform3D(Basis(q), origin);
}

Array NT4::get_pose3d_array(String topic, Array default_val) {
  std::string key = topic.utf8().get_data();
  std::vector<uint8_t> data = get_raw_bytes(inst, raw_subs, key);

  // Each pose is 7 doubles = 56 bytes
  if (data.size() % 56 != 0)
    return default_val;

  size_t count = data.size() / 56;
  Array result;

  // Pre-allocate (if possible, though Array implementation might manage this)
  // result.resize(count);

  const uint8_t *ptr = data.data();

  for (size_t i = 0; i < count; i++) {
    // 1. Unpack FRC Position
    double tx, ty, tz;
    std::memcpy(&tx, ptr, 8);
    std::memcpy(&ty, ptr + 8, 8);
    std::memcpy(&tz, ptr + 16, 8);

    // 2. Unpack FRC Rotation
    double qw, qx, qy, qz;
    std::memcpy(&qw, ptr + 24, 8);
    std::memcpy(&qx, ptr + 32, 8);
    std::memcpy(&qy, ptr + 40, 8);
    std::memcpy(&qz, ptr + 48, 8);

    // 3. Convert to Godot Coordinates
    // Pos: (-y, z, -x)
    Vector3 origin(-ty, tz, -tx);

    // Rot: (-y, z, -x, w)
    Quaternion q(-qy, qz, -qx, qw);

    // Godot Transform3D
    result.append(Transform3D(Basis(q), origin));

    // Advance pointer
    ptr += 56;
  }

  return result;
}

Array NT4::get_topic_info() {
  std::vector<nt::TopicInfo> topics = inst.GetTopicInfo();
  Array godot_topics;
  for (const auto &info : topics) {
    Dictionary entry;
    entry["name"] = String(info.name.c_str());
    entry["type"] = String(info.type_str.c_str());
    godot_topics.append(entry);
  }
  return godot_topics;
}

void NT4::_bind_methods() {
  ClassDB::bind_method(D_METHOD("start_client", "server_ip"),
                       &NT4::start_client);

  ClassDB::bind_method(D_METHOD("get_number", "topic", "default"),
                       &NT4::get_number);
  ClassDB::bind_method(D_METHOD("set_number", "topic", "value"),
                       &NT4::set_number);

  // Updated bind for Array
  ClassDB::bind_method(D_METHOD("get_number_array", "topic", "default"),
                       &NT4::get_number_array);
  ClassDB::bind_method(D_METHOD("set_number_array", "topic", "value"),
                       &NT4::set_number_array);

  ClassDB::bind_method(D_METHOD("get_boolean", "topic", "default"),
                       &NT4::get_boolean);
  ClassDB::bind_method(D_METHOD("set_boolean", "topic", "value"),
                       &NT4::set_boolean);

  ClassDB::bind_method(D_METHOD("get_string", "topic", "default"),
                       &NT4::get_string);
  ClassDB::bind_method(D_METHOD("set_string", "topic", "value"),
                       &NT4::set_string);

  ClassDB::bind_method(D_METHOD("get_boolean_array", "topic", "default"),
                       &NT4::get_boolean_array);
  ClassDB::bind_method(D_METHOD("set_boolean_array", "topic", "value"),
                       &NT4::set_boolean_array);

  ClassDB::bind_method(D_METHOD("get_string_array", "topic", "default"),
                       &NT4::get_string_array);
  ClassDB::bind_method(D_METHOD("set_string_array", "topic", "value"),
                       &NT4::set_string_array);

  ClassDB::bind_method(D_METHOD("set_string_array", "topic", "value"),
                       &NT4::set_string_array);

  ClassDB::bind_method(D_METHOD("get_value", "topic", "default"),
                       &NT4::get_value);
  ClassDB::bind_method(D_METHOD("subscribe_to_all"), &NT4::subscribe_to_all);

  ClassDB::bind_method(D_METHOD("get_translation2d", "topic", "default"),
                       &NT4::get_translation2d);
  ClassDB::bind_method(D_METHOD("get_rotation2d", "topic", "default"),
                       &NT4::get_rotation2d);
  ClassDB::bind_method(D_METHOD("get_pose2d", "topic", "default"),
                       &NT4::get_pose2d);

  ClassDB::bind_method(D_METHOD("get_translation3d", "topic", "default"),
                       &NT4::get_translation3d);
  ClassDB::bind_method(D_METHOD("get_rotation3d", "topic", "default"),
                       &NT4::get_rotation3d);
  ClassDB::bind_method(D_METHOD("get_pose3d", "topic", "default"),
                       &NT4::get_pose3d);
  ClassDB::bind_method(D_METHOD("get_pose3d_array", "topic", "default"),
                       &NT4::get_pose3d_array);

  ClassDB::bind_method(D_METHOD("get_topic_info"), &NT4::get_topic_info);
}