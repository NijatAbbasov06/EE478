import os
import qrcode
from PIL import Image

input_path = os.path.join(os.path.dirname(__file__), "..", "resources", "quiz.txt")
output_base = os.path.join(os.path.dirname(__file__), "..", "models")

template_sdf = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="qr_panel">
      <pose>0 0 0 0 0 -1.5708</pose>
      <visual name="qr_visual">
        <geometry>
          <box><size>0.5 0.01 0.5</size></box>
        </geometry>
        <material>
          <script>
            <uri>model://{model_name}/materials/scripts</uri>
            <uri>model://{model_name}/materials/textures</uri>
            <name>{material_name}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

template_material = """material {material_name}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture {texture_name}
        filtering none
      }}
    }}
  }}
}}
"""

with open(input_path, "r") as f:
    content = f.read()

quizzes = content.strip().split("# ")

i = 0
for block in quizzes:
    if not block.strip():
        continue
    i += 1
    lines = block.strip().split("\n")
    model_name = f"qr_code_{i:02d}"
    material_name = f"quiz_{i}"
    texture_name = f"{material_name}.png"
    model_dir = os.path.join(output_base, model_name)
    scripts_dir = os.path.join(model_dir, "materials", "scripts")
    textures_dir = os.path.join(model_dir, "materials", "textures")

    os.makedirs(scripts_dir, exist_ok=True)
    os.makedirs(textures_dir, exist_ok=True)

    quiz_text = "\n".join(lines[1:]).strip()
    qr_img_path = os.path.join(textures_dir, texture_name)
    qrcode.make(quiz_text).save(qr_img_path)
    img = img.resize((1024, 1024), Image.NEAREST) 

    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(template_sdf.format(model_name=model_name, material_name=material_name))

    with open(os.path.join(scripts_dir, f"{material_name}.material"), "w") as f:
        f.write(template_material.format(material_name=material_name, texture_name=texture_name))

    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0" ?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
</model>
""")

    print(f"✅ Generated: {model_name}")
