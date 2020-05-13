
#include <libs/logging/SqliteTable.hpp>

#include <iostream>

SqliteTable::SqliteTable(const std::string &table_name)
  : _db(nullptr), _table_name(table_name)
{
}

SqliteTable::~SqliteTable(void)
{
  _db.reset();
  sqlite3_shutdown();
}

bool SqliteTable::open_database(const std::string &database_file)
{
  sqlite3 *db_local = nullptr;

  int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;
  std::string full_path = std::string(kDatabaseDirectory) + database_file;
  int r = sqlite3_open_v2(full_path.c_str(), &db_local, flags, nullptr);
  if(r == SQLITE_OK){
    _db = std::shared_ptr<sqlite3>(db_local, [](sqlite3 *p) {sqlite3_close(p); } );
  } else {
    // TODO: Raise error status? Exception?
    return false;
  }

  return true;
}

bool SqliteTable::create_table(void) {
  if(!_db || _table_name.empty()) { return false; }

  std::string cmd = "CREATE TABLE IF NOT EXISTS " + _table_name;
  cmd += " (timestamp REAL, test_time_s REAL, x REAL, theta REAL)";

  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, nullptr) != SQLITE_OK) {
    // TODO: error
    sqlite3_finalize(st);
    return false;
  }

  if(sqlite3_step(st) != SQLITE_DONE) {
    // TODO: error
    sqlite3_finalize(st);
    return false;
  }

  _table_exists = true;
  sqlite3_finalize(st);
  return true;
}

void SqliteTable::subscribe(ros::NodeHandle &nh, std::string &topic)
{
  topic.insert(0, "/");
  std::cout << "Subscribe to " << topic << std::endl;
  _subscriber = nh.subscribe(topic, 100, &SqliteTable::callback, this);
}

void SqliteTable::callback(const pendulum::State::ConstPtr &msg)
{
  double timestamp = msg->header.stamp.toSec();
  double theta     = msg->theta;
  double x         = msg->x;
  double test_time = (msg->header.stamp - _test_start_time).toSec();

  if(!insert_row(timestamp, test_time, x, theta)) {
      ROS_WARN("Failure to insert row into database table");
  }
}

void SqliteTable::set_start_time(ros::Time start_time)
{
  _test_start_time = start_time;
}

bool SqliteTable::insert_row(const double timestamp, const double test_time, const double x, const double theta) {
  if(!_db || !_table_exists) { return false; }

  std::string cmd = "INSERT INTO " + _table_name;
  cmd += " (timestamp, test_time_s, x, theta) VALUES (:ts, :tt, :x, :theta)";

  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, nullptr) != SQLITE_OK) {
    // TODO: error
    sqlite3_finalize(st);
    return false;
  }

  sqlite3_bind_double(st, sqlite3_bind_parameter_index(st, ":ts"), timestamp);
  sqlite3_bind_double(st, sqlite3_bind_parameter_index(st, ":tt"), test_time);
  sqlite3_bind_double(st, sqlite3_bind_parameter_index(st, ":x"), x);
  sqlite3_bind_double(st, sqlite3_bind_parameter_index(st, ":theta"), theta);

  if(sqlite3_step(st) != SQLITE_DONE) {
    sqlite3_finalize(st);
    return false;
  };

  sqlite3_finalize(st);
  return true;
}
