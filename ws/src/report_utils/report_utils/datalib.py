from pathlib import Path

SAVE_DIR = Path(".")


def create_save_directories():
    if not SAVE_DIR.exists():
        print("Creating save directory")
        SAVE_DIR.mkdir(parents=True)


def set_save_directories(dirname: str):
    global SAVE_DIR
    SAVE_DIR = Path(dirname)
    create_save_directories()


def open_(filename, *args, **kwargs):
    return open(SAVE_DIR / filename, *args, **kwargs)
