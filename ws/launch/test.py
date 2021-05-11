import matplotlib.pyplot as plt
import yaml

with open("launch/bagsaver.yaml", "r") as file:
    data = yaml.safe_load(file)


print(data)

topiclist = list(data["topics"].keys())

for topic, d in data["topics"].items():
    mod = __import__(d["topic_type_module"], fromlist=[d["topic_type"]])
    topic_type = getattr(mod, d["topic_type"])


for plotname, plotdata in data["plots"].items():
    pn, pd = plotname, plotdata
    print("Plot config")
    plt.plot([1, 2], [1, 2])

    if pd["type"] == "timeseries":
        print(pd["ylabel"])
        plt.ylabel(pd["ylabel"])

    plt.xlim(0, 5)
    plt.ylim(0, 5)
plt.show()
