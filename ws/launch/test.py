import yaml

with open("launch/bagsaver_config.yaml", "r") as file:
    data = yaml.safe_load(file)

topiclist = list(data["topics"].keys())

for topic, d in data["topics"].items():
    mod = __import__(d["topic_type_module"], fromlist=[d["topic_type"]])
    topic_type = getattr(mod, d["topic_type"])
    t = topic_type()
    print(t)
