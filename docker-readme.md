To run docker container, first build:
docker build -t humble .

Then run: 
docker run -it --net=host --ipc=host humble