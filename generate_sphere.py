import math

def create_icosphere(subdivisions=2, filename="sphere.obj"):
    # Golden ratio for initial icosahedron vertices
    phi = (1 + math.sqrt(5)) / 2
    
    # 1. Initial 12 vertices of a regular icosahedron
    verts = [
        (-1,  phi,  0), ( 1,  phi,  0), (-1, -phi,  0), ( 1, -phi,  0),
        ( 0, -1,  phi), ( 0,  1,  phi), ( 0, -1, -phi), ( 0,  1, -phi),
        ( phi,  0, -1), ( phi,  0,  1), (-phi,  0, -1), (-phi,  0,  1)
    ]
    
    # Normalize vertices to make it a unit sphere (radius = 1)
    def normalize(v):
        length = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        return (v[0]/length, v[1]/length, v[2]/length)

    verts = [normalize(v) for v in verts]

    # Initial 20 faces
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)
    ]

    # 2. Subdivision helper
    midpoint_cache = {}
    def get_midpoint(v1_idx, v2_idx):
        key = tuple(sorted((v1_idx, v2_idx)))
        if key in midpoint_cache:
            return midpoint_cache[key]
        
        v1, v2 = verts[v1_idx], verts[v2_idx]
        mid = normalize(((v1[0]+v2[0])/2, (v1[1]+v2[1])/2, (v1[2]+v2[2])/2))
        verts.append(mid)
        index = len(verts) - 1
        midpoint_cache[key] = index
        return index

    # 3. Perform Subdivisions
    for _ in range(subdivisions):
        new_faces = []
        for tri in faces:
            v1, v2, v3 = tri
            a = get_midpoint(v1, v2)
            b = get_midpoint(v2, v3)
            c = get_midpoint(v3, v1)
            
            new_faces.extend([(v1, a, c), (v2, b, a), (v3, c, b), (a, b, c)])
        faces = new_faces

    # 4. Write to OBJ
    with open(filename, "w") as f:
        f.write("# Icosphere OBJ\n")
        for v in verts:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for tri in faces:
            # OBJ is 1-indexed
            f.write(f"f {tri[0]+1} {tri[1]+1} {tri[2]+1}\n")

    print(f"Generated {filename} with {len(faces)} faces.")

if __name__ == "__main__":
    # Increase subdivisions for a smoother sphere (e.g., 3 or 4)
    create_icosphere(subdivisions=3)
