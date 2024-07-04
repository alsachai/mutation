import carla

client = carla.Client('localhost', 2000)
client.reload_world()
record_path = 'home/tieriv/alsachai/recording'
print(client.show_recorder_file_info(record_path))
client.replay_file(record_path,0,0,0)