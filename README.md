bag_convert
---


# Install
```shell
# clone repository
git clone https://github.com/tier4/bag_converter.git

# clone dependencies
cd bag_converter
vcs import src < repos.yaml

# build
cd docker
./build.sh

# clean build
./build.sh --no-cache
```


# Example
```shell
./bag_converter <path-to-intput-mcap> <path-to-output-mcap>
```