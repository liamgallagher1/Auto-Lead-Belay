#include "rope_feeder.hpp"

int main(int argc, char *argv[])
{
  if (argc < 2) {
    cout << "Include an argument 0 if you don't want to output a log or 1 if you want to output a log" << endl;
    return 1;
  }
  
  bool make_log = strcmp(argv[1], "0");
  string file_name;
  if (make_log) {
    file_name = string(argv[2]);
  }

  RopeFeeder feeder(make_log, file_name);
}

