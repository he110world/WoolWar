using UnityEngine;
using System.Collections;
using IK;

public class NewBehaviourScript : MonoBehaviour {
	public Plane plane;
	public GameObject go;
	public Object cube;
	public Transform skeleton;
	public float fuck;
	//public Mesh mesh;
	
	Mesh buildMesh()
	{
		// You can change that line to provide another MeshFilter
		MeshFilter filter = gameObject.AddComponent< MeshFilter >();
		Mesh mesh = filter.mesh;
		mesh.Clear();
		 
		float length = 3f;
		float width = 3f;
		float height = 3f;
		 
		#region Vertices
		Vector3 p0 = new Vector3( -length * .5f,	-width * .5f, height * .5f );
		Vector3 p1 = new Vector3( length * .5f, 	-width * .5f, height * .5f );
		Vector3 p2 = new Vector3( length * .5f, 	-width * .5f, -height * .5f );
		Vector3 p3 = new Vector3( -length * .5f,	-width * .5f, -height * .5f );	
		 
		Vector3 p4 = new Vector3( -length * .5f,	width * .5f,  height * .5f );
		Vector3 p5 = new Vector3( length * .5f, 	width * .5f,  height * .5f );
		Vector3 p6 = new Vector3( length * .5f, 	width * .5f,  -height * .5f );
		Vector3 p7 = new Vector3( -length * .5f,	width * .5f,  -height * .5f );
		 
		Vector3[] vertices = new Vector3[]
		{
			// Bottom
			p0, p1, p2, p3,
		 
			// Left
			p7, p4, p0, p3,
		 
			// Front
			p4, p5, p1, p0,
		 
			// Back
			p6, p7, p3, p2,
		 
			// Right
			p5, p6, p2, p1,
		 
			// Top
			p7, p6, p5, p4
		};
		#endregion
		 
		#region Normales
		Vector3 up 	= Vector3.up;
		Vector3 down 	= Vector3.down;
		Vector3 front 	= Vector3.forward;
		Vector3 back 	= Vector3.back;
		Vector3 left 	= Vector3.left;
		Vector3 right 	= Vector3.right;
		 
		Vector3[] normales = new Vector3[]
		{
			// Bottom
			down, down, down, down,
		 
			// Left
			left, left, left, left,
		 
			// Front
			front, front, front, front,
		 
			// Back
			back, back, back, back,
		 
			// Right
			right, right, right, right,
		 
			// Top
			up, up, up, up
		};
		#endregion	
		 
		#region UVs
		Vector2 _00 = new Vector2( 0f, 0f );
		Vector2 _10 = new Vector2( 1f, 0f );
		Vector2 _01 = new Vector2( 0f, 1f );
		Vector2 _11 = new Vector2( 1f, 1f );
		 
		Vector2[] uvs = new Vector2[]
		{
			// Bottom
			_11, _01, _00, _10,
		 
			// Left
			_11, _01, _00, _10,
		 
			// Front
			_11, _01, _00, _10,
		 
			// Back
			_11, _01, _00, _10,
		 
			// Right
			_11, _01, _00, _10,
		 
			// Top
			_11, _01, _00, _10,
		};
		#endregion
		 
		#region Triangles
		int[] triangles = new int[]
		{
			// Bottom
			3, 1, 0,
			3, 2, 1,			
		 
			// Left
			3 + 4 * 1, 1 + 4 * 1, 0 + 4 * 1,
			3 + 4 * 1, 2 + 4 * 1, 1 + 4 * 1,
		 
			// Front
			3 + 4 * 2, 1 + 4 * 2, 0 + 4 * 2,
			3 + 4 * 2, 2 + 4 * 2, 1 + 4 * 2,
		 
			// Back
			3 + 4 * 3, 1 + 4 * 3, 0 + 4 * 3,
			3 + 4 * 3, 2 + 4 * 3, 1 + 4 * 3,
		 
			// Right
			3 + 4 * 4, 1 + 4 * 4, 0 + 4 * 4,
			3 + 4 * 4, 2 + 4 * 4, 1 + 4 * 4,
		 
			// Top
			3 + 4 * 5, 1 + 4 * 5, 0 + 4 * 5,
			3 + 4 * 5, 2 + 4 * 5, 1 + 4 * 5,
		 
		};
		#endregion
		 
		mesh.vertices = vertices;
		mesh.normals = normales;
		mesh.uv = uvs;
		mesh.triangles = triangles;
		 
		mesh.RecalculateBounds();
		mesh.Optimize();		
		
		return mesh;
	}
	
	void test(){
	    gameObject.AddComponent<Animation>();
	    SkinnedMeshRenderer renderer = gameObject.AddComponent<SkinnedMeshRenderer>();
	 
	 	//Social
	    
		/*MESH*/
		//cube = Resources.Load("Cube");
	    //Mesh sourceMesh = Instantiate(cube) as Mesh;
		Mesh sourceMesh = buildMesh();
	    //sourceMesh.RecalculateNormals();
	 
	    BoneWeight[] weights = new BoneWeight[sourceMesh.vertexCount];
	    for(int i = 0; i < weights.Length; i++){
			float rnd = Random.value;
			int idx = Random.Range(1,3);
	       weights[i].boneIndex0 = idx;
			weights[i].boneIndex1 = idx==1?2:1;
	       weights[i].weight0 = rnd;//1;
			weights[i].weight1 = 1f - rnd;
	    }
	    sourceMesh.boneWeights = weights;
	 
	 
	 
	    /*SKELETON*/
	    Transform[] bones;
	    Transform sourceSkeleton = Instantiate(skeleton) as Transform;
	 
	    sourceSkeleton.name = "skeleton";
	    sourceSkeleton.parent = gameObject.transform;
	 
	    bones = gameObject.GetComponentsInChildren<Transform>();
	 
	 
	 
	    /*BIND POSES*/
	    Matrix4x4[] bindPoses = new Matrix4x4[bones.Length];
	    for(int i = 0; i < bindPoses.Length; i++){
	       bindPoses[i] = bones[i].worldToLocalMatrix * transform.localToWorldMatrix;
	    }
	 
	    /*MATERIAL*/
	    Material[] materials = new Material[3];
	    materials[0] = materials[1] = materials[2] = new Material(Shader.Find(" Diffuse"));
	    renderer.materials = materials;
	 
	    /*END*/
	    sourceMesh.bindposes = bindPoses;
	    renderer.bones = bones;
	    renderer.sharedMesh = sourceMesh;
	}
	// Use this for initialization
	void Start () {
		//plane = new Plane(Vector3.up,Vector3.zero);
		//Instantiate(Vector3.zero, Quaternion.identity);
		//go = Mesh
		test ();
		Debug.Log(fuck);
	}
	
	// Update is called once per frame
	void Update () {
	
	}
	
	void OnGUI(){
		
	}
}
