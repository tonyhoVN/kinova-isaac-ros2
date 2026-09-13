"""Shared lighting helpers for generated Isaac Sim stages."""

from __future__ import annotations

from env.config import SCENE_DOME_LIGHT, SCENE_SPHERE_LIGHTS, WORLD


def create_soft_light_rig(stage) -> None:
    """Add balanced ambient, key, and fill lights around the work table."""
    from pxr import Gf, UsdGeom, UsdLux

    lighting_path = f"{WORLD}/Lighting"
    UsdGeom.Xform.Define(stage, lighting_path)

    # Ambient illumination keeps surfaces facing away from the key light visible.
    dome = UsdLux.DomeLight.Define(stage, f"{lighting_path}/AmbientDome")
    dome.CreateIntensityAttr(SCENE_DOME_LIGHT["intensity"])
    dome.CreateColorAttr(Gf.Vec3f(*SCENE_DOME_LIGHT["color"]))
    dome.CreateSpecularAttr(SCENE_DOME_LIGHT["specular"])
    UsdLux.ShadowAPI.Apply(dome.GetPrim()).CreateShadowEnableAttr(False)

    # Broad non-shadowing fills soften the one natural-looking overhead shadow.
    for settings in SCENE_SPHERE_LIGHTS:
        light = UsdLux.SphereLight.Define(
            stage, f"{lighting_path}/{settings['name']}"
        )
        light.CreateIntensityAttr(settings["intensity"])
        light.CreateRadiusAttr(settings["radius"])
        light.CreateColorAttr(Gf.Vec3f(*settings["color"]))
        light.CreateSpecularAttr(settings["specular"])
        UsdLux.ShadowAPI.Apply(light.GetPrim()).CreateShadowEnableAttr(
            settings["cast_shadows"]
        )
        translate = UsdGeom.Xformable(light.GetPrim()).AddTranslateOp(
            precision=UsdGeom.XformOp.PrecisionDouble
        )
        translate.Set(Gf.Vec3d(*settings["position"]))
