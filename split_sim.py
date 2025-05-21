import os

def split_file(file_path: str, max_size: int, output_dir: str) -> None:
    """
    Divide el fichero en partes de hasta max_size bytes.
    """
    os.makedirs(output_dir, exist_ok=True)
    base_name = os.path.basename(file_path)
    part_num = 1

    with open(file_path, 'rb') as source_file:
        while True:
            chunk = source_file.read(max_size)
            if not chunk:
                break
            part_name = f"{base_name}.part{part_num:03d}"
            part_path = os.path.join(output_dir, part_name)
            with open(part_path, 'wb') as part_file:
                part_file.write(chunk)
            print(f"Wrote {part_path} ({len(chunk)} bytes)")
            part_num += 1

if __name__ == "__main__":
    # Fichero de log que quieres partir
    file_path = r"sim.log"
    # Directorio de salida (misma carpeta)
    output_dir = r"."
    # 5 MB en bytes
    max_size = 5 * 1024 * 1024

    split_file(file_path, max_size, output_dir)
