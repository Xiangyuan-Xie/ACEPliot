#include <flight_logger.hpp>
#include <rosbag2_storage/storage_factory.hpp>
#include <algorithm>
#include <ctime>

FlightLogger::FlightLogger()
: FlightLogger(RosbagParams{})
{
}

FlightLogger::FlightLogger(const RosbagParams & bagp)
: bag_p_(bagp)
{
  // Normalize the base topic so later concatenation always yields valid paths.
  bag_p_.base_topic = sanitize_key(bag_p_.base_topic);
  try {
    // Generate a timestamp for the log directory with millisecond precision.
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &time_t);
#else
    local_tm = *std::localtime(&time_t);
#endif

    std::stringstream timestamp_ss;
    timestamp_ss << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S")
                 << "_" << std::setw(3) << std::setfill('0') << ms.count();
    std::string timestamp = timestamp_ss.str();

    // Default output directory is <root>/logs, overridable via parameters.
    std::filesystem::path logs_dir;
    if (bag_p_.root_dir.empty()) {
      std::filesystem::path root_dir;
#ifdef ROOT_DIR
      root_dir = std::filesystem::path(ROOT_DIR);
#else
      root_dir = std::filesystem::current_path();
#endif
      logs_dir = root_dir / "logs";
    } else {
      logs_dir = std::filesystem::path(bag_p_.root_dir) / "logs";
    }

    // If the directory already exists, append an incrementing suffix to avoid collisions.
    std::filesystem::path log_file = logs_dir / timestamp;
    if (std::filesystem::exists(log_file)) {
      int suffix = 1;
      while (std::filesystem::exists(logs_dir / (timestamp + "_" + std::to_string(suffix)))) {
        ++suffix;
      }
      log_file = logs_dir / (timestamp + "_" + std::to_string(suffix));
    }
    std::string log_path = log_file.string();

    // Prefer configured storage backend and fall back from mcap to sqlite3 if needed.
    std::string storage_id = bag_p_.storage_id;
    try {
      rosbag2_storage::StorageOptions sopt;
      sopt.uri = log_path;
      sopt.storage_id = storage_id;
      sopt.max_bagfile_size = bag_p_.max_bagfile_size;
      sopt.max_bagfile_duration = bag_p_.max_bagfile_duration;

      rosbag2_cpp::ConverterOptions copt;
      copt.input_serialization_format = "cdr";
      copt.output_serialization_format = "cdr";

      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      writer_->open(sopt, copt);
    } catch (const std::exception &) {
      if (storage_id == "mcap") {
        storage_id = "sqlite3";
        rosbag2_storage::StorageOptions sopt;
        sopt.uri = log_path;
        sopt.storage_id = storage_id;
        sopt.max_bagfile_size = bag_p_.max_bagfile_size;
        sopt.max_bagfile_duration = bag_p_.max_bagfile_duration;

        rosbag2_cpp::ConverterOptions copt;
        copt.input_serialization_format = "cdr";
        copt.output_serialization_format = "cdr";

        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open(sopt, copt);
      } else {
        throw;
      }
    }
  } catch (const std::exception &) {
    throw;
  }

#if defined(ROS_DISTRO_ROLLING) || defined(ROS_DISTRO_IRON) || defined(ROS_DISTRO_JAZZY)
  if (bag_p_.enable_compression) {
    rosbag2_cpp::CompressionOptions comp;
    comp.compression_mode = rosbag2_cpp::compression_modes::FILE;
    comp.compression_format = "zstd";
    writer_->set_compression_options(comp);
  }
#endif
}

FlightLogger::~FlightLogger()
{
  close();
}

void FlightLogger::close()
{
  // Idempotent close path; safe to call multiple times.
  if (writer_) {
    try {
      writer_->close();
      writer_.reset();
    } catch (const std::exception &) {
    }
  }
}

bool FlightLogger::isOpen() const
{
  return writer_ != nullptr;
}

std::string FlightLogger::topic_(const std::string & sub_topic) const
{
  // Normalize topic shape to "/base/sub".
  std::string base = bag_p_.base_topic;
  if (base.empty() || base[0] != '/') {
    base = "/" + base;
  }
  if (base.length() > 1 && base.back() == '/') {
    base.pop_back();
  }

  std::string clean_sub = sub_topic;
  if (!clean_sub.empty() && clean_sub[0] == '/') {
    clean_sub = clean_sub.substr(1);
  }

  return base + "/" + clean_sub;
}

std::string FlightLogger::sanitize_key(std::string k)
{
  // Filter invalid characters and lowercase the key for rosbag-safe topic naming.
  std::string s;
  s.reserve(k.size());
  for (char c : k) {
    char lower_c = std::tolower(static_cast<unsigned char>(c));
    if ((lower_c >= 'a' && lower_c <= 'z') ||
      (lower_c >= '0' && lower_c <= '9') ||
      lower_c == '_' || lower_c == '/')
    {
      s.push_back(lower_c);
    } else {
      s.push_back('_');
    }
  }

  if (!s.empty() && s[0] != '/') {
    s.erase(0, s.find_first_not_of('_'));
  }

  while (!s.empty() && s.back() == '/') {
    s.pop_back();
  }

  if (s.empty() || std::isdigit(static_cast<unsigned char>(s[0]))) {
    s.insert(0, "k_");
  }
  return s;
}
