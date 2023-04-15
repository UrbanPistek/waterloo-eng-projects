# info

Launch Carla Locally: 
```
docker-compose --file carla.yaml up
```

# Connect to Foxglove instance Running on a Server

```
ssh -L 5900:localhost:8765 wato-server
```

Connect foxglove studio to:

```
ws://localhost:5900
```
