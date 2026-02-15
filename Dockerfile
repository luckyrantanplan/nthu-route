FROM ubuntu:24.04 AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        libboost-all-dev \
        zlib1g-dev \
        libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)"

RUN cd build && ctest --output-on-failure

FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
        libboost-all-dev \
        zlib1g \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=builder /app/build/NthuRoute /usr/local/bin/NthuRoute
COPY --from=builder /app/POWV9.dat /app/POST9.dat /app/

ENTRYPOINT ["NthuRoute"]
CMD ["--help"]
