cargo clean
# docker build -t marshal .
docker build -t marshal .
# docker run --rm marshal
docker run --rm -it --entrypoint bash marshal