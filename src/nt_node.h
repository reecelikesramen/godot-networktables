#ifndef NT_NODE_H
#define NT_NODE_H

#include <godot_cpp/classes/node.hpp>
#include <networktables/BooleanArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/RawTopic.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>

// Godot Types
#include <godot_cpp/variant/packed_float64_array.hpp> // <--- NEW: Native Array
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/transform2d.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

// Additional Godot Types
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <map>
#include <string>
#include <vector>

namespace godot {

class NT4 : public Node {
  GDCLASS(NT4, Node)

private:
  nt::NetworkTableInstance inst;

  // STORAGE
  std::map<std::string, nt::DoubleSubscriber> double_subs;
  std::map<std::string, nt::DoubleArraySubscriber> double_array_subs;
  std::map<std::string, nt::RawSubscriber> raw_subs;
  std::map<std::string, nt::BooleanSubscriber> boolean_subs;
  std::map<std::string, nt::StringSubscriber> string_subs;
  std::map<std::string, nt::BooleanArraySubscriber> boolean_array_subs;
  std::map<std::string, nt::StringArraySubscriber> string_array_subs;

  // FIX: This was the cause of your compile error!
  std::map<std::string, nt::DoublePublisher> double_pubs;
  std::map<std::string, nt::DoubleArrayPublisher> double_array_pubs;
  std::map<std::string, nt::BooleanPublisher> boolean_pubs;
  std::map<std::string, nt::StringPublisher> string_pubs;
  std::map<std::string, nt::BooleanArrayPublisher> boolean_array_pubs;
  std::map<std::string, nt::StringArrayPublisher> string_array_pubs;

public:
  NT4();
  ~NT4();

  void start_client(String server_ip);

  double get_number(String topic, double default_val);
  void set_number(String topic, double value);

  // FIX: Return PackedFloat64Array instead of std::vector to prevent ABI
  // crashes
  PackedFloat64Array get_number_array(String topic,
                                      PackedFloat64Array default_val);
  void set_number_array(String topic, PackedFloat64Array value);

  bool get_boolean(String topic, bool default_val);
  void set_boolean(String topic, bool value);

  String get_string(String topic, String default_val);
  void set_string(String topic, String value);

  Array get_boolean_array(String topic, Array default_val);
  void set_boolean_array(String topic, Array value);

  PackedStringArray get_string_array(String topic,
                                     PackedStringArray default_val);
  void set_string_array(String topic, PackedStringArray value);

  // Geometry
  Vector2 get_translation2d(String topic, Vector2 default_val);
  double get_rotation2d(String topic, double default_val);
  Transform2D get_pose2d(String topic, Transform2D default_val);

  Vector3 get_translation3d(String topic, Vector3 default_val);
  Quaternion get_rotation3d(String topic, Quaternion default_val);
  Transform3D get_pose3d(String topic, Transform3D default_val);

  // Discovery
  Array get_topic_info();

protected:
  static void _bind_methods();
};

} // namespace godot

#endif