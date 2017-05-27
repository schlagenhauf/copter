protoc --plugin=protoc-gen-nanopb=/home/pheenx/code/extern/nanopb/generator/protoc-gen-nanopb --nanopb_out=. copcom.proto

protoc --python_out=./remote_interface copcom.proto
