import yaml

with open("launch/bagsaver.yaml", "r") as file:
    data = yaml.safe_load(file)


print(data)

topiclist = list(data["topics"].keys())

for topic, d in data["topics"].items():
    mod = __import__(d["topic_type_module"], fromlist=[d["topic_type"]])
    topic_type = getattr(mod, d["topic_type"])

    print(d["time_attribute"])
    print(d["data_attributes"])
