# syntax=docker/dockerfile:1
# Build stage. Needs the network to install the toolchain; pin the base image so
# the finalist rebuild produces the same binary.
FROM debian:bookworm-slim AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends g++ cmake make ca-certificates \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY CMakeLists.txt ./
COPY src ./src
COPY third_party ./third_party
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build --parallel

# Runtime stage: no compiler, just the binary. The match runs this as uid 65534
# with a read-only disk and 1 CPU, so nothing here may need root or write disk.
FROM debian:bookworm-slim
COPY --from=build /src/build/bot /bot
ENTRYPOINT ["/bot"]
