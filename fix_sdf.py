import os

def fix_sdf_files(models_dir):
    for root, dirs, files in os.walk(models_dir):
        for f in files:
            if f.endswith('.sdf') or f.endswith('.config'):
                filepath = os.path.join(root, f)
                with open(filepath, 'r', encoding='utf-8') as file:
                    content = file.read()
                
                if 'model://aws_robomaker_' in content:
                    new_content = content.replace('model://aws_robomaker_', 'model://')
                    with open(filepath, 'w', encoding='utf-8') as file:
                        file.write(new_content)
                    print(f"Fixed {filepath}")

if __name__ == '__main__':
    models_dir = 'src/Intelligent Autonomous Hospital Delivery-world/models'
    fix_sdf_files(models_dir)
    print("Done fixing SDFs.")
