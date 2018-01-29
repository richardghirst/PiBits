FROM arm32v6/alpine as builder
RUN apk add --update build-base autoconf automake linux-headers
ADD . /pi-blaster
WORKDIR /pi-blaster
RUN ./autogen.sh
RUN ./configure
RUN make

FROM arm32v6/alpine
COPY --from=builder /pi-blaster/pi-blaster .
CMD ["./pi-blaster", "-D"]
