#pragma once
#include<io.h>
#include<cstdlib>
#include<iostream>
#include<string.h>
#include<fstream>
#include<vector>
#include<sstream>
#include ".\MeshLib\core\Mesh\Vertex.h"
#include ".\MeshLib\core\Mesh\Edge.h"
#include ".\MeshLib\core\Mesh\Face.h"
#include ".\MeshLib\core\Mesh\HalfEdge.h"
#include ".\MeshLib\core\Mesh\BaseMesh.h"
#include ".\MeshLib\core\Parser\parser.h"
#include ".\MeshLib\core\Mesh\iterators.h"
#include ".\MeshLib\toolbox\toolbox.h"

using namespace MeshLib;
using std::string;
using std::vector;
class CVertexColor :public CVertex {
public:
	float colorsRGB[3];
	CPoint normal;
	bool hasColor = false;
	void _from_string();
};

void CVertexColor::_from_string() {
	static std::stringstream _sstream;
	_sstream.str("");
	_sstream.clear();
	CParser parser(this->string());
	for (auto tokenIter = parser.tokens().begin(); tokenIter != parser.tokens().end(); ++tokenIter) {
		CToken *pToken = *tokenIter;
		if (pToken->m_key == "rgb") {
			std::string values = pToken->m_value;
			values.erase(values.begin());
			values.erase(values.end() - 1);
			_sstream << values;
			_sstream >> colorsRGB[0] >> colorsRGB[1] >> colorsRGB[2];
		}
	}
}
typedef CBaseMesh<CVertexColor, CEdge, CFace, CHalfEdge> CMesh;
typedef MeshToolBox<CVertexColor, CEdge, CFace, CHalfEdge> CToolBox;

void compute_normal(CMesh * pMesh)
{
	for (CToolBox::MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
	{
		CVertexColor * v = *viter;
		CPoint n(0, 0, 0);
		for (CToolBox::VertexFaceIterator vfiter(v); !vfiter.end(); ++vfiter)
		{
			CFace * pF = *vfiter;

			CPoint p[3];
			CHalfEdge * he = pF->halfedge();
			for (int k = 0; k < 3; k++)
			{
				p[k] = he->target()->point();
				he = he->he_next();
			}

			CPoint fn = (p[1] - p[0]) ^ (p[2] - p[0]);
			//pF->normal() = fn / fn.norm();
			n += fn;
		}

		n = n / n.norm();
		v->normal = n;
	}
};
void getFiles(string path, vector<string>& files)
{
	//文件句柄  
	long   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				//if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
void readMFile(CMesh *pmesh, string filePath){
	pmesh->read_m(filePath.c_str());
	compute_normal(pmesh);
}