
INCLUDE_DIRECTORIES( . )

SET(jsoncpp_SRC
  json_reader.cpp
  json_value.cpp
  json_writer.cpp 
)

ADD_LIBRARY( jsoncpp EXCLUDE_FROM_ALL ${jsoncpp_SRC})
