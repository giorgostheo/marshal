cargo clean
# docker build -t marshal .
docker build -t marshal .
# docker run --rm -it --entrypoint bash marshal
docker run --rm marshal cargo run -- opw_tr --release > result.txt