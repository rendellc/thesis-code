import yaml

import sys


def main():
    if len(sys.argv) != 2:
        print("Need a config file")
        print(sys.argv[0], "experiments.yaml")
        return

    experiment_config_file = sys.argv[1]
    with open(experiment_config_file, "r") as file:
        config = yaml.safe_load(file)

    print(config)


if __name__ == "__main__":
    main()
