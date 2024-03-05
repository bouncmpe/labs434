import pathlib

from dm_control import mjcf

def get_model(file, ASSETS=dict()):

    model_path = pathlib.Path(file)
    assets_dir = model_path.parent / 'assets'

    if assets_dir.exists() and not ASSETS:
        for item in assets_dir.iterdir():
            ASSETS[item.name] = item.read_bytes()

    model = mjcf.from_path(model_path, assets=ASSETS)

    return model, model.get_assets()

