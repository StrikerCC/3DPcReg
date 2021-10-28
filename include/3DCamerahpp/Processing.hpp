#ifndef __PROCESSING_HPP__
#define __PROCESSING_HPP__

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <memory.h>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <algorithm>
#include "Types.hpp"
#define VALIDATE(x) (x < 90000.f)

namespace cs
{
	typedef struct float3 { 
		float x, y, z;
		float3() { x = 0;  y = 0; z = 0; };
		float3(float a, float b, float c){x = a; y=b; z=c;}
	}float3;
	
	inline float3 operator + (const float3 & a, const float3 & b) { return float3(a.x + b.x, a.y + b.y, a.z + b.z); }
	inline float3 operator - (const float3 & a, const float3 & b) { return float3(a.x - b.x, a.y - b.y, a.z - b.z); }
	inline float3 operator * (const float3 & a, float b) { return float3(a.x * b, a.y * b, a.z * b); }
	typedef struct float2{
		float u, v;
		float2(){};
		float2(float a, float b){u = a;v = b;}
	}float2;
/**
* @~chinese
* \defgroup Process ���ͼ����
* @brief ���ͼ����ӿ�
* @{
* @~english
* \defgroup Process Depth map's common operation
* @brief Camera Object Interface Class
* @{
*/

	/*!\class Pointcloud
	* @~chinese
	* @brief ������
	* @~english 
	* @brief Pointcloud class 
	**/
	class Pointcloud
	{
	public:
		Pointcloud(){};
		~Pointcloud(){};

		/**
		* @~chinese
		* @brief �����ͼ����Ϊ����
		* @param[in] depthMap			���ͼ��ַ
		* @param[in] width				���ͼ���
		* @param[in] height				���ͼ�߶�
		* @param[in] depthScale			������ݵ�����ϵ��,ͨ��ICamera::getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, ...)���
		* @param[in] intrinsicsDepth	��������ڲ�, ͨ��ICamera::getIntrinsic(STREAM_TYPE_DEPTH, ...)���
		* @param[in] intrinsicsRgb		RGB���ڲ�,ͨ��ICamera::getIntrinsics(STREAM_TYPE_RGB, ...)��ã�
		* @								���������������nullptr
		* @param[in] extrinsics			�������RGB������תƽ�ƾ���, ͨ��ICamera::getExtrinsics���
		*								���������������nullptr
		* @param[in] removeInvalid		true: �Ƴ���Ч��, false: ����Ч������Ϊ(0,0,0)
		* @~english
		* @brief Calculate pointcloud from depth map
		* @param[in] depthMap			The address of the depth map
		* @param[in] width				The width of the depth map
		* @param[in] height				The height of the depth map
		* @param[in] depthScale			The depthScale of the depth, return by ICamera::getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, ...)
		* @param[in] intrinsicsDepth	The intrinsics of the depth stream, return by ICamera::getIntrinsics(STREAM_TYPE_DEPTH, ...)
		* @param[in] intrinsicsRgb		The intrinsics of the RGB stream, return by ICamera::getIntrinsics(STREAM_TYPE_RGB, ...)
		* @								it should be setted to nullptr if no texture
		* @param[in] extrinsics			The extrinsics of the RGB stream, return by ICamera::getExtrinsics
		* @								it should be setted to nullptr if no texture
		* @param[in] removeInvalid		true: remove invalid point, false: set invalid point to (0,0,0)
		*/
		void generatePoints(unsigned short *depthMap, int width, int height, float depthScale,
                            Intrinsics *intrinsicsDepth, Intrinsics *intrinsicsRgb, Extrinsics *extrinsics, bool removeInvalid = false)
		{
			_points.clear();
			_textures.clear();
			_normals.clear();
			float3 *points = (float3 *)malloc(width * height * sizeof(float3));
			float3 *normals = (float3 *)malloc(width * height * sizeof(float3));
			float3 *smoothNormal = (float3 *)malloc(width * height * sizeof(float3));

			Intrinsics intrDepth;
			memcpy(&intrDepth, intrinsicsDepth, sizeof(Intrinsics));
			intrDepth.fx *= float(width) / intrinsicsDepth->width;
			intrDepth.cx *= float(width) / intrinsicsDepth->width;
			intrDepth.fy *= float(height) / intrinsicsDepth->height;
			intrDepth.cy *= float(height) / intrinsicsDepth->height;
			
			int index = 0;
			for(int v = 0; v < height; v++)
			{
				for(int u = 0; u < width; u++, index++)
				{
					float z = depthMap[index] * depthScale;
					if(z > 0.f)
					{
						points[index].x = (u - intrDepth.cx) * z / intrDepth.fx;
						points[index].y = (v - intrDepth.cy) * z / intrDepth.fy;
						points[index].z = z;
					}
					else
					{
						points[index].x = points[index].y = points[index].z = 0.0;
					}

					normals[index].x = normals[index].y = normals[index].z = 0.0;
				}
			}

			calculateNormals(points, normals, width, height);

			smoothNormals(width, height, points, normals, smoothNormal, 2);

			insertPoints(points, normals, width, height, extrinsics, intrinsicsRgb, removeInvalid);
			
			free(points);
			free(normals);
			free(smoothNormal);
		}

		/**
		* @~chinese
		* @brief ����rgbͼ��ת�������ͼ��С��λ�õ����ͼλ�õ�ӳ���
		* @		 �����ȵ���generatePoints������rgb����Ĳ�������removeInvalid����Ϊfalse�����������꣬���򷵻�ʧ��
		* @param[in]  depthWidth		���ͼ���
		* @param[in]  depthHeight		���ͼ�߶�
		* @param[out] rgbToDepth		���ӳ���
		* @return     �ɹ�:true, ʧ��:false
		* @~english
		* @brief Generate a map of RGB image (converted to depth map size) coords to depth map coords
		* @		 You must call generatePoints and pass in rgb parameters, and removeInvalid is set to false to generate texture coordinates, otherwise false is returned
		* @param[in]  depthWidth		The width of the depth map
		* @param[in]  depthHeight		The height of the depth map
		* @param[out] rgbToDepth		output the map rgb coord to depth coord
		* @return     �ɹ�:true, ʧ��:false
		*/
		bool generateTextureToDepthMap(int depthWidth, int depthHeight, std::map<int, int> &rgbToDepth)
		{
			int length = depthHeight*depthWidth;
			if (!_textures.size() || _textures.size() != length)
				return false;
			int rgbIndex;
			for (size_t i = 0; i < length; i++)
			{
				rgbIndex = _textures[i].u * depthWidth + _textures[i].v * depthHeight * depthWidth;
				if (rgbIndex > 0)
				{
					rgbToDepth[rgbIndex] = i;
				}
			}
		}

		/**
		* @~chinese
		* @brief ����rgb�����Ӧ�����ͼ����
		* @		 �����ȵ���generateTextureToDepthMap����map��
		* @param[in]  rgbToDepth		ӳ���
		* @param[in]  rgbX				rgbͼX����
		* @param[in]  rgbY				rgbͼY����
		* @param[in]  rgbWidth			rgbͼ���
		* @param[in]  rgbHeight			rgbͼ�߶�
		* @param[in]  depthWidth		���ͼ���
		* @param[in]  depthHeight		���ͼ�߶�
		* @param[out] depthX			������ͼX����
		* @param[out] depthY			������ͼY����
		* @return     �ɹ�:true, ʧ��:false
		* @~english
		* @brief Calculate the depth map coordinates corresponding to the RGB coordinates
		* @		 You must call generateTextureToDepthMap to generate map first
		* @param[in]  rgbToDepth		the map rgb coordinates to depth map coordinates
		* @param[in]  rgbX				The x-coordinates of rgb image
		* @param[in]  rgbY				The y-coordinates of rgb image
		* @param[in]  rgbWidth			The width of the rgb image
		* @param[in]  rgbHeight			The height of the rgb image
		* @param[in]  depthWidth		The width of the depth map
		* @param[in]  depthHeight		The height of the depth map
		* @param[out] depthX			Output the x-coordinates of depth map
		* @param[out] depthY			Output the y-coordinates of depth map
		* @return    �ɹ�:true, ʧ��:false
		*/
		bool getDepthCoordFromMap(std::map<int, int> &rgbToDepth, int rgbX, int rgbY, int rgbWidth, int rgbHeight,
			int depthWidth, int depthHeight, int &depthX, int &depthY)
		{
			rgbX = float(rgbX) / rgbWidth * depthWidth;
			rgbY = float(rgbY) / rgbHeight * depthHeight;
			int targetIndex, x, y;
			int index = 0;
			int xoffsets[] = { 0, 1, -1, 2, -2 };
			int yoffsets[] = { 0, 1, -1 };
			bool found = false;
			for (auto yoff : yoffsets)
			{
				y = rgbY + yoff;
				if (y >= 0 && y < depthHeight)
				{
					for (auto xoff : xoffsets)
					{
						x = rgbX + xoff;
						if (x >= 0 && x < depthWidth)
						{
							targetIndex = y*depthWidth + x;
							if (rgbToDepth.find(targetIndex) != rgbToDepth.end())
							{
								index = rgbToDepth[targetIndex];
								found = true;
								break;
							}
						}
					}
					if (found)
						break;
				}
			}
			if (found)
			{
				depthX = index % depthWidth;
				depthY = index / depthWidth;
			}
			return found;
		}
	
		/**
		* @~chinese
		* @brief �������ͼ�ϵ�һ����
		* @param[out] vertex			���ض�������
		* @param[out] textureCoord		����RGBͼ���ӳ������(�ѹ�һ����0~1.f)������RGBͼ��ķֱ��ʼ��ɵõ���������ֵ
		* @param[in] u					���ͼ�ϵ�������
		* @param[in] v					���ͼ�ϵ�������
		* @param[in] depth				���ͼ����ָ��
		* @param[in] width				���ͼ���
		* @param[in] height				���ͼ�߶�
		* @param[in] depthScale			���ͼһ����λ��ʾ�������ȣ���λ���ף�
		* @param[in] intrinsicsDepth	��������ڲ�, ͨ��ICamera::getIntrinsic(STREAM_TYPE_DEPTH, ...)���
		* @param[in] intrinsicsRgb		RGB���ڲ�,ͨ��ICamera::getIntrinsics(STREAM_TYPE_RGB, ...)��ã�
		* @								���������������nullptr
		* @param[in] extrinsics			�������RGB������תƽ�ƾ���, ͨ��getExtrinsics���
		*								���������������nullptr
		* @~english
		* @brief Calculate one point
		* @param[out] vertex			return coordinate of point
		* @param[out] textureCoord		return texture coordinate of point(0~1.f), multiply the image width/height to get the coordinates in image
		* @param[in] u					The x-coordinate of the depth map
		* @param[in] v					The y-coordinate of the depth map
		* @param[in] depth				the pointer of depth map
		* @param[in] width				The width of the depth map
		* @param[in] height				The height of the depth map
		* @param[in] depthScale			the physical length represented by a unit
		* @param[in] intrinsicsDepth	The intrinsics of the depth stream, return by ICamera::getIntrinsics(STREAM_TYPE_DEPTH, ...)
		* @param[in] intrinsicsRgb		The intrinsics of the RGB stream, return by ICamera::getIntrinsics(STREAM_TYPE_RGB, ...)
		* @								it should be setted to nullptr if no texture
		* @param[in] extrinsics			The extrinsics of the RGB stream, return by ICamera::getExtrinsics
		*								it should be setted to nullptr if no texture
		**/
		void generatePoint(float3& vertex, float2& textureCoord, int u, int v, unsigned short depth, int width,
                           int height, float depthScale,Intrinsics *intrinsicsDepth,
                           Intrinsics *intrinsicsRgb = nullptr, Extrinsics *extrinsics = nullptr)
		{
			Intrinsics intrDepth;
			memcpy(&intrDepth, intrinsicsDepth, sizeof(Intrinsics));
			float scaleX = float(width) / intrinsicsDepth->width;
			float scaleY = float(height) / intrinsicsDepth->height;
			intrDepth.fx *= scaleX;
			intrDepth.cx *= scaleX;
			intrDepth.fy *= scaleY;
			intrDepth.cy *= scaleY;

			vertex = float3(0.f, 0.f, 0.f);
			textureCoord = float2(0.f, 0.f);

			float z = depth * depthScale;
			if (z > 0.f)
			{
				vertex.x = (u - intrDepth.cx) * z / intrDepth.fx;
				vertex.y = (v - intrDepth.cy) * z / intrDepth.fy;
				vertex.z = z;
				if (extrinsics && intrinsicsRgb)
				{
					float3 tmp = vertex;
					tmp.x += extrinsics->translation[0];
					tmp.y += extrinsics->translation[1];
					tmp.z += extrinsics->translation[2];
					float3 tmp2;
					tmp2.x = tmp.x * extrinsics->rotation[0] + tmp.y * extrinsics->rotation[1] + tmp.z * extrinsics->rotation[2];
					tmp2.y = tmp.x * extrinsics->rotation[3] + tmp.y * extrinsics->rotation[4] + tmp.z * extrinsics->rotation[5];
					tmp2.z = tmp.x * extrinsics->rotation[6] + tmp.y * extrinsics->rotation[7] + tmp.z * extrinsics->rotation[8];

					float fu = (intrinsicsRgb->fx * tmp2.x / tmp2.z + intrinsicsRgb->cx);
					float fv = (intrinsicsRgb->fy * tmp2.y / tmp2.z + intrinsicsRgb->cy);
					if (fu >= 0.00001f && fu < intrinsicsRgb->width && fv >= 0.00001f && fv < intrinsicsRgb->height)
					{
						textureCoord = float2(fu / intrinsicsRgb->width, fv / intrinsicsRgb->height);
					}
				}
			}
		}

		/**
		* @~chinese
		* @brief ���������ݵ������ļ�
		* @param[in] filename			�����ļ���
		* @param[in] texture			RGBͼ��ĵ�ַ�����������ɫ�����
		* @param[in] textureWidth		RGBͼ��Ŀ��
		* @param[in] textureHeight		RGBͼ��ĸ߶�
		* @param[in] binary				���ݸ�ʽ, ������/�ı�
		* @~english
		* @brief Export pointcloud to file
		* @param[in] filename			Target save filename
		* @param[in] texture			The address of RGB image, it should be setted to nullptr if no texture
		* @param[in] textureWidth		The width of RGB image
		* @param[in] textureHeight		The height of RGB image
		* @param[in] binary				File format, Binary/Ascii
		**/
		void exportToFile(const std::string& filename, unsigned char *texture, int textureWidth, int textureHeight, bool binary = false)
		{
			const auto vertices = getVertices();
			const auto texcoords = getTexcoords();
			const auto normals = getNormals();

			std::string ext = filename.substr(filename.size() - 3, 3);
			std::string fileName = filename.substr(0, filename.size() - 4);
			if (ext == "ply")
			{
				std::ofstream out(filename);
				out << "ply\n";
				if (binary)
					out << "format binary_little_endian 1.0\n";
				else
					out << "format ascii 1.0\n";
				out << "comment pointcloud saved from 3DCamera\n";
				out << "element vertex " << size() << "\n";
				out << "property float x\n";
				out << "property float y\n";
				out << "property float z\n";
				out << "property float nx\n";
				out << "property float ny\n";
				out << "property float nz\n";
				if (texture)
				{
					out << "property uchar red\n";
					out << "property uchar green\n";
					out << "property uchar blue\n";
				}
				out << "end_header\n";
				out.close();

				if (binary)
				{
					out.open(filename, std::ios_base::app | std::ios_base::binary);
					for (int i = 0; i < size(); ++i)
					{
						// we assume little endian architecture on your device
						out.write(reinterpret_cast<const char*>(&(vertices[i].x)), sizeof(float));
						out.write(reinterpret_cast<const char*>(&(vertices[i].y)), sizeof(float));
						out.write(reinterpret_cast<const char*>(&(vertices[i].z)), sizeof(float));
						out.write(reinterpret_cast<const char*>(&(normals[i].x)), sizeof(float));
						out.write(reinterpret_cast<const char*>(&(normals[i].y)), sizeof(float));
						out.write(reinterpret_cast<const char*>(&(normals[i].z)), sizeof(float));

						if (texture)
						{
							int x, y;
							x = int(texcoords[i].u * textureWidth);
							if (x >= textureWidth) x = textureWidth - 1;
							if (x < 0)	x = 0;
							y = int(texcoords[i].v * textureHeight);
							if (y >= textureHeight) y = textureHeight - 1;
							if (y < 0)	y = 0;
							unsigned char *color = texture + (y * textureWidth + x) * 3;
							uint8_t r = color[0];
							uint8_t g = color[1];
							uint8_t b = color[2];
							out.write(reinterpret_cast<const char*>(&r), sizeof(uint8_t));
							out.write(reinterpret_cast<const char*>(&g), sizeof(uint8_t));
							out.write(reinterpret_cast<const char*>(&b), sizeof(uint8_t));
						}
					}
					out.close();
				}
				else
				{

					out.open(filename, std::ios_base::app);
					for (int i = 0; i < size(); ++i)
					{
						// we assume little endian architecture on your device
						out << (vertices[i].x) << " ";
						out << (vertices[i].y) << " ";
						out << (vertices[i].z) << " ";
						out << (normals[i].x) << " ";
						out << (normals[i].y) << " ";
						out << (normals[i].z) << " ";

						if (texture)
						{
							int x, y;
							x = int(texcoords[i].u * textureWidth);
							if (x >= textureWidth) x = textureWidth - 1;
							if (x < 0)	x = 0;
							y = int(texcoords[i].v * textureHeight);
							if (y >= textureHeight) y = textureHeight - 1;
							if (y < 0)	y = 0;
							unsigned char *color = texture + (y * textureWidth + x) * 3;
							uint8_t r = color[0];
							uint8_t g = color[1];
							uint8_t b = color[2];
							out << int(r) << " ";
							out << int(g) << " ";
							out << int(b) << " ";
						}
						out << "\n";
					}
					out.close();
				}
			}
			else if (ext == "csv")
			{
				FILE *fpout;
				fpout = fopen(filename.c_str(), "w+");
				//out.open(filename, std::ios_base::app);
				for (int i = 0; i < size(); ++i)
				{
					fprintf(fpout, "%.6f,%.6f,%.6f \n", vertices[i].x, vertices[i].y, vertices[i].z);
				}
				fclose(fpout);
			}
			else
			{
				FILE *fpout;
				fpout = fopen(filename.c_str(), "wb");
				FILE *fmap;
				fmap = fopen((fileName + ".map").c_str(), "wb");
				for (int i = 0; i < vertices.size(); ++i)
				{
					if (vertices[i].z <= 0.f)
						continue;
					fprintf(fpout, "v %.6f %.6f %.6f \n", vertices[i].x, vertices[i].y, vertices[i].z);
					fprintf(fpout, "vn %.6f %.6f %.6f \n", normals[i].x, normals[i].y, normals[i].z);
					if (texture)
					{
						fprintf(fmap, "%d %d\n", int(texcoords[i].u * textureWidth), textureHeight - int(texcoords[i].v * textureHeight));
					}
				}
				fclose(fpout);
				fclose(fmap);
			}
		}

		/**
		* @~chinese
		* @brief ��ȡ�ܵ���
		* @~english
		* @brief Return the size of points(inclue size of invalid points)
		**/
		int size()
		{
			return int(_points.size());
		}
		/**
		* @~chinese
		* @brief ��ȡ��Ч�ĵ���
		* @~english
		* @brief Return the size of valid points(exclude size of invalid points)
		**/
		int validSize()
		{
			return _validSize;
		}
		/**
		* @~chinese
		* @brief ��ȡ���ƵĶ�������
		* @~english
		* @brief Return the vertices of pointcloud
		**/
		std::vector<float3> & getVertices()
		{
			return _points;
		}

		/**
		* @~chinese
		* @brief ��ȡ��Ӧ��RGBͼ���ϵ�����(�ѹ�һ����0~1.f)������RGBͼ��ķֱ��ʼ��ɵõ���������ֵ
		* @~english
		* Return the corresponding texture coordinates of pointcloud (0~1.f), multiply the image width/height to get the coordinates in image
		*/
		std::vector<float2> &getTexcoords()
		{
			return _textures;
		}
		/**
		* @~chinese
		* @brief ��ȡ����
		* @~english
		* @brief Return the normals of pointcloud.
		**/
		std::vector<float3> &getNormals()
		{
			return _normals;
		}

	private:
		float3 al_vector_cross(float3 a, float3 b)
		{
			float3 c;

			c.x = a.y*b.z - a.z*b.y;
			c.y = a.z*b.x - a.x*b.z;
			c.z = a.x*b.y - a.y*b.x;

			return c;
		}

		void calculateNormals(float3 *points, float3 *normals, int width, int height)
		{
			float3 normal, e1, e2;
			float3 p0, p1, p2;
			float3 *n0, *n1, *n2;
			int w = width - 1;
			int h = height - 1;
			for (int u = 0; u < w; ++u)
			{
				for (int v = 0; v < h; ++v)
				{
					p0 = points[u + v * width];
					p1 = points[u + (v + 1) * width];
					p2 = points[u + 1 + (v + 1)* width];
					n0 = &normals[u + v * width];
					n1 = &normals[u + (v + 1) * width];
					n2 = &normals[u + 1 + (v + 1) * width];

					if (p0.z > 0.1f && p1.z > 0.1f && p2.z > 0.1f)
					{
						float depth_threshold = 5.f;

						float z0 = fabs(p2.z - p1.z);
						float z1 = fabs(p2.z - p0.z);
						float z2 = fabs(p0.z - p1.z);

						if (z0 > depth_threshold || z1 > depth_threshold || z2 > depth_threshold)
						{
							normal.x = 0.0;
							normal.y = 0.0;
							normal.z = 0.0;
						}
						else
						{
							e1.x = p1.x - p0.x;
							e1.y = p1.y - p0.y;
							e1.z = p1.z - p0.z;
							e2.x = p2.x - p0.x;
							e2.y = p2.y - p0.y;
							e2.z = p2.z - p0.z;
							normal = al_vector_cross(e1, e2);
						}
						n0->x += normal.x;
						n0->y += normal.y;
						n0->z += normal.z;
						n1->x += normal.x;
						n1->y += normal.y;
						n1->z += normal.z;
						n2->x += normal.x;
						n2->y += normal.y;
						n2->z += normal.z;
					}
				}
			}


			for (int u = 0; u < width - 1; u++)
			{
				for (int v = 0; v < height - 1; v++)
				{
					p0 = points[u + v * width];
					p1 = points[u + 1 + v * width];
					p2 = points[u + 1 + (v + 1)* width];
					n0 = &normals[u + v * width];
					n1 = &normals[u + 1 + v * width];
					n2 = &normals[u + 1 + (v + 1) * width];

					if (p0.z > 0.1 && p1.z > 0.1 && p2.z > 0.1)
					{
						float depth_threshold = 5.f;

						float z0 = fabs(p2.z - p1.z);
						float z1 = fabs(p2.z - p0.z);
						float z2 = fabs(p0.z - p1.z);

						if (z0 > depth_threshold || z1 > depth_threshold || z2 > depth_threshold)
						{
							normal.x += 0.0;
							normal.y += 0.0;
							normal.z += 0.0;
						}
						else
						{
							e1.x = p1.x - p0.x;
							e1.y = p1.y - p0.y;
							e1.z = p1.z - p0.z;
							e2.x = p2.x - p0.x;
							e2.y = p2.y - p0.y;
							e2.z = p2.z - p0.z;
							normal = al_vector_cross(e2, e1);
						}
						n0->x += normal.x;
						n0->y += normal.y;
						n0->z += normal.z;
						n1->x += normal.x;
						n1->y += normal.y;
						n1->z += normal.z;
						n2->x += normal.x;
						n2->y += normal.y;
						n2->z += normal.z;
					}
				}
			}

		}

		float3 normalizeVector(const float3 &vector)
		{
			float l = sqrtf(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
			if (l < 0.00001) {
				return vector;
			}
			else {
				float il = 1 / l;
				return float3(vector.x * il, vector.y * il, vector.z * il );
			}
		}

		int smoothNormals(int width, int height,
			const float3* points,
			float3* inNormal,
			float3* outNormal,
			int radius)
		{
			int size = width * height;
			// normalize
			for (size_t ui = 0; ui < size; ui++)
			{
				float3 p = points[ui];
				if (p.z > 0.1) {
					inNormal[ui] = normalizeVector(inNormal[ui]);
				}
			}

			int u = 0, v = 0;
			int i = 0, j = 0;
			for (v = 0; v < height; v++) {
				for (u = 0; u < width; u++) {
					i = j + u;
					float3 p = points[i];
					if (p.z > 0.1) {
						int dw = 0, tw = 0;
						float3 sn(0, 0, 0);
						for (int rv = -radius; rv <= radius; rv++) {
							for (int ru = -radius; ru <= radius; ru++) {
								int dv = v + rv;
								int du = u + ru;

								if (dv < 0) continue;
								if (du < 0) continue;
								if (dv >= height) continue;
								if (du >= width) continue;

								int ii = du + dv*width;
								float3 tp = points[ii];
								if (tp.z < 0.1) continue;

								dw = 2 * ((radius + 1 - abs(rv)) + (radius + 1 - abs(ru)));
								float3 tn = inNormal[ii];
								sn.x += tn.x * dw;
								sn.y += tn.y * dw;
								sn.z += tn.z * dw;
								tw += dw;
							}
						}
						if (tw > 0) {
							outNormal[i] = normalizeVector(float3(sn.x / tw, sn.y / tw, sn.z / tw));
						}
					}
				}
				j += width;
			}
			return 0;
		}

		void insertPoints(float3 *points, float3 *normals, int width, int height, Extrinsics *extrinsics, Intrinsics *intrinsicsRgb, bool removeInvalid)
		{
			_validSize = 0;
			int index = 0;
			_textures.reserve(width * height);
			_points.reserve(width * height);
			_normals.reserve(width * height);
			for (int v = 0; v < height; ++v)
			{
				for (int u = 0; u < width; ++u, ++index)
				{
					float z = points[index].z;
					float x = points[index].x;
					float y = points[index].y;
					float3 pt(x,y,z);
					float3 tmp,tmp2;

					if (extrinsics && intrinsicsRgb)
					{
						tmp = pt;
						tmp.x += extrinsics->translation[0];
						tmp.y += extrinsics->translation[1];
						tmp.z += extrinsics->translation[2];
						tmp2.x = tmp.x * extrinsics->rotation[0] + tmp.y * extrinsics->rotation[1] + tmp.z * extrinsics->rotation[2];
						tmp2.y = tmp.x * extrinsics->rotation[3] + tmp.y * extrinsics->rotation[4] + tmp.z * extrinsics->rotation[5];
						tmp2.z = tmp.x * extrinsics->rotation[6] + tmp.y * extrinsics->rotation[7] + tmp.z * extrinsics->rotation[8];

						if (z > 0.f)
						{
							int fu = int(intrinsicsRgb->fx * tmp2.x / tmp2.z + intrinsicsRgb->cx);
							int fv = int(intrinsicsRgb->fy * tmp2.y / tmp2.z + intrinsicsRgb->cy);

							if (fu >= 0.00001 && fu < intrinsicsRgb->width && fv >= 0.00001 && fv < intrinsicsRgb->height)
							{
								_textures.emplace_back(float2(float(fu) / intrinsicsRgb->width, float(fv) / intrinsicsRgb->height));
								_points.emplace_back(pt);
								_normals.emplace_back(normals[u + v * width]);
								_validSize++;
							}
							else if (!removeInvalid)
							{
								_textures.emplace_back(float2(0.f, 0.f));
								_points.emplace_back(float3(0.f, 0.f, 0.f));
								_normals.emplace_back(float3(0.f, 0.f, 0.f));;
							}
						}
						else if (!removeInvalid)
						{
							_textures.emplace_back(float2(0.f, 0.f));
							_points.emplace_back(float3(0.f, 0.f, 0.f));
							_normals.emplace_back(float3(0.f, 0.f, 0.f));;
						}
						
					}
					else
					{
						if (pt.z > 0.f || !removeInvalid)
						{
							_points.emplace_back(pt);
							_normals.emplace_back(normals[index]);
							_textures.emplace_back(float2(float(u) / width, float(v) / height));
							if (pt.z > 0.f)
								_validSize++;
						}
					}
				}
			}
		}

	private:
		std::vector<float3> _points;
		std::vector<float2> _textures;
		std::vector<float3> _normals;
		int _validSize;
	};

	/**
	* @~chinese
	* @brief �����ͼת��ΪRGBͼ
	* @~english
	* @brief Convert depth map to RGB image
	**/
	class colorizer
	{
	public:
		colorizer() :_zmin(0), _zmax(5000)
		{
			initColormap(4000);
		}
		/**
		* @~chinese
		* @brief      ������Ⱦɫ����Ӧ��������ֵ����С���ֵ
		* @param[in]  zmin				����С���ֵ
		* @param[in]  zmax				��������ֵ
		* @~english
		* @brief      set maximum depth and minimum depth corresponding to color map
		* @param[in]  zmin				��the minimum depth
		* @param[in]  zmax				��the maximum depth
		**/
		void setRange(int zmin, int zmax)
		{
			_zmin = zmin;
			_zmax = zmax;
		}

		/**
		* @~chinese
		* @brief      �����ͼת��ΪRGBͼ
		* @param[in]  depthData			���������
		* @param[in]  scale				���������ϵ��
		* @param[in]  rgbData			��RGB���ݱ����ַ
		* @param[in]  dataSize			��Ԫ������
		* @~english
		* @brief      set maximum depth and minimum depth corresponding to color map
		* @param[in]  depthData			��the pointer of depth data
		* @param[in]  scale				��the scale of depth
		* @param[in]  rgbData			��the pointer of RGB data
		* @param[in]  dataSize			��element size
		**/
		template < typename T >
		void process(T* depthData, float scale, unsigned char *rgbData, int dataSize)
		{
			T max = _zmax;
			T min = _zmin;
			T range = max - min;
			for (auto i = 0; i < dataSize; ++i)
			{
				auto d = depthData[i] * scale;
				if (d>0)
				{
					auto f = range != 0 ? (d - min)*1.0 / range : 0;
					auto c = getColor(f);
					rgbData[i * 3 + 0] = (uint8_t)c.x;
					rgbData[i * 3 + 1] = (uint8_t)c.y;
					rgbData[i * 3 + 2] = (uint8_t)c.z;
				}
				else
				{
					rgbData[i * 3 + 0] = 0;
					rgbData[i * 3 + 1] = 0;
					rgbData[i * 3 + 2] = 0;
				}
			}
		}
	private:

		inline float3 getColor(float value) const
		{
			if (value < 0) value = 0;
			if (value > 1) value = 1;
			return _cacheData[(int)(value * (_cacheSize - 1))];
		}
		inline float3 lerp(const float3& a, const float3& b, float t) const
		{
			float3 v = b * t + a * (1 - t);
			if (v.x < 0) v.x = 0;
			if (v.x > 255) v.x = 255;
			if (v.y < 0) v.y = 0;
			if (v.y > 255) v.y = 255;
			if (v.z < 0) v.z = 0;
			if (v.z > 255) v.z = 255;
			return v;
		}

		float3 calc(float value) const
		{
			// if we have exactly this value in the map, just return it
			if (_map.find(value) != _map.end()) return _map.at(value);

			auto lower = _map.lower_bound(value) == _map.begin() ? _map.begin() : --(_map.lower_bound(value));
			auto upper = _map.upper_bound(value);

			auto t = (value - lower->first) / (upper->first - lower->first);
			auto c1 = lower->second;
			auto c2 = upper->second;
			return lerp(c1, c2, t);
		}

		void initColormap(int steps)
		{
			_map[0] = float3(0, 0, 255);
			_map[0.25f] = float3(0, 255, 255);
			_map[0.50f] = float3(255, 255, 0);
			_map[0.75f] = float3(255, 0, 0);
			_map[1.00f] = float3(50, 0, 0);

			_cache.resize(steps + 1);
			for (int i = 0; i <= steps; i++) {
				auto t = (float)i / steps;
				_cache[i] = calc(t);
			}

			_cacheSize = _cache.size();
			_cacheData = _cache.data();
		}

	private:
		int _zmin;
		int _zmax;
		std::map<float, float3> _map;
		std::vector<float3> _cache;
		size_t _cacheSize; float3* _cacheData;
	};

	namespace filter
	{
	/**
	* @~chinese
	* @brief ö��: �˲���ʽ
	* @~english
	* @brief enumeration: type of filter
	**/
	typedef enum
	{
		FILTER_TYPE_AVERAGE = 0x00,
		FILTER_TYPE_MEDIAN = 0x01,
		FILTER_TYPE_GAUSSIAN = 0x02,
	}FilterType;

	static void generateGaussianKernel(float *gaussianKernel, int filterSize, int sigma)
	{
		int m = filterSize / 2;
		int n = filterSize / 2;

		float s = 2.0f * sigma*sigma;
		float sum = 0;

		for (int i = 0; i < filterSize; i++)
		{
			for (int j = 0; j < filterSize; j++)
			{
				int x = i - m;
				int y = j - n;

				gaussianKernel[j*filterSize + i] = float(exp(-(x*x + y*y) / s) / (M_PI*s));
			}
		}

		for (int i = 0; i < filterSize * filterSize; i++)
		{
			sum += gaussianKernel[i];
		}

		for (int i = 0; i < filterSize * filterSize; i++)
		{
			gaussianKernel[i] = gaussianKernel[i] / sum;
		}
	}

	static void generateAverageKernel(float *averageKernel, int filterSize)
	{
		memset(averageKernel, 0, sizeof(float) * filterSize * filterSize);
		for (int i = 0; i < filterSize * filterSize; i++)
		{
			averageKernel[i] = 1.0f / (filterSize * filterSize);
		}
	}
	/**
	* @~chinese
	* @brief      ��˹�˲�
	* @param[in]  depthData			���������
	* @param[in]  width				��ͼ����
	* @param[in]  height			��ͼ��߶�
	* @param[in]  filterSize		��ģ��ߴ磬����Ϊ��������ֵԽ���˲��̶�Խ��
	* @param[in]  sigma				��ϵ��
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Execute gaussian filter
	* @param[in]  depthData			��pointer of data
	* @param[in]  width				��the width of image
	* @param[in]  height			��the height of image
	* @param[in]  filterSize		��the size of filter template��it must be odd 
	* @param[in]  sigma				��sigma
	* @return success:return 0, fail:other error code
	**/
	template <typename T>
	static int GaussianBlur(T *depthData, int width, int height, int filterSize, int sigma)
	{
		T *filterData = NULL;
		float *kernelData = NULL;
		T *calcData = NULL;

		filterData = (T *)malloc(sizeof(T) * width * height);
		memset(filterData, 0, sizeof(T) * width * height);

		calcData = (T*)malloc(sizeof(T) * filterSize * filterSize);
		memset(calcData, 0, sizeof(T) * filterSize * filterSize);

		kernelData = (float *)malloc(sizeof(float) * filterSize * filterSize);
		memset(kernelData, 0, sizeof(T) * filterSize * filterSize);

		generateGaussianKernel(kernelData, filterSize, sigma);

		for (int h = filterSize / 2; h < height - filterSize / 2; h++)
		{
			for (int w = filterSize / 2; w < width - filterSize / 2; w++)
			{
				bool flag = true;
				for (int i = 0; i < filterSize; i++)
				{
					if (!flag)
						break;

					for (int j = 0; j < filterSize; j++)
					{
						if (depthData[(h + i - filterSize / 2) * width + w + j - filterSize / 2] < 1)
						{
							flag = false;
							break;
						}
						else
						{
							calcData[i* filterSize + j] = depthData[(h + i - filterSize / 2) * width + w + j - filterSize / 2];
						}
					}
				}

				if (flag)
				{
					float tmp_data = 0;
					for (int k = 0; k < filterSize * filterSize; k++)
					{
						tmp_data += (calcData[k] * kernelData[k]);
					}
					filterData[h * width + w] = tmp_data;
				}
			}
		}

		int count = 0;

		for (int h = 0; h < height; h++)
		{
			for (int w = 0; w < width; w++)
			{
				if (filterData[h*width + w] > 1)
				{
					depthData[h*width + w] = filterData[h*width + w];
					count++;
				}
			}
		}
		free(filterData);
		filterData = NULL;

		free(calcData);
		calcData = NULL;

		if (kernelData)
		{
			free(kernelData);
			kernelData = NULL;
		}
	}
	/**
	* @~chinese
	* @brief      ��ֵ�˲�
	* @param[in]  depthData			���������
	* @param[in]  width				��ͼ����
	* @param[in]  height			��ͼ��߶�
	* @param[in]  filterSize		��ģ��ߴ磬����Ϊ��������ֵԽ���˲��̶�Խ��
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Execute average filter
	* @param[in]  depthData			��pointer of data
	* @param[in]  width				��the width of image
	* @param[in]  height			��the height of image
	* @param[in]  filterSize		��the size of filter template��it must be odd 
	* @return success:return 0, fail:other error code
	**/
	template <typename T>
	static int AverageBlur(T *depthData, int width, int height, int filterSize)
	{
		T *filterData = NULL;
		float *kernelData = NULL;
		T *calcData = NULL;

		filterData = (T *)malloc(sizeof(T) * width * height);
		memset(filterData, 0, sizeof(T) * width * height);

		calcData = (T*)malloc(sizeof(T) * filterSize * filterSize);
		memset(calcData, 0, sizeof(T) * filterSize * filterSize);

		kernelData = (float *)malloc(sizeof(float) * filterSize * filterSize);
		memset(kernelData, 0, sizeof(T) * filterSize * filterSize);

		generateAverageKernel(kernelData, filterSize);

		for (int h = filterSize / 2; h < height - filterSize / 2; h++)
		{
			for (int w = filterSize / 2; w < width - filterSize / 2; w++)
			{
				bool flag = true;
				for (int i = 0; i < filterSize; i++)
				{
					if (!flag)
						break;

					for (int j = 0; j < filterSize; j++)
					{
						if (depthData[(h + i - filterSize / 2) * width + w + j - filterSize / 2] < 1)
						{
							flag = false;
							break;
						}
						else
						{
							calcData[i* filterSize + j] = depthData[(h + i - filterSize / 2) * width + w + j - filterSize / 2];
						}
					}
				}

				if (flag)
				{
					float tmp_data = 0;
					for (int k = 0; k < filterSize * filterSize; k++)
					{
						tmp_data += (calcData[k] * kernelData[k]);
					}
					filterData[h * width + w] = tmp_data;
				}
			}
		}

		int count = 0;

		for (int h = 0; h < height; h++)
		{
			for (int w = 0; w < width; w++)
			{
				if (filterData[h*width + w] > 1)
				{
					depthData[h*width + w] = filterData[h*width + w];
					count++;
				}
			}
		}
		free(filterData);
		filterData = NULL;

		free(calcData);
		calcData = NULL;

		if (kernelData)
		{
			free(kernelData);
			kernelData = NULL;
		}
		return 0;
	}

	/**
	* @~chinese
	* @brief      ��ֵ�˲�
	* @param[in]  depthData			���������
	* @param[in]  width				��ͼ����
	* @param[in]  height			��ͼ��߶�
	* @param[in]  filterSize		��ģ��ߴ磬����Ϊ��������ֵԽ���˲��̶�Խ��
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Execute median filter
	* @param[in]  depthData			��pointer of data
	* @param[in]  width				��the width of image
	* @param[in]  height			��the height of image
	* @param[in]  filterSize		��the size of filter template��it must be odd 
	* @return success:return 0, fail:other error code
	**/
	template <typename T>
	static int MedianBlur(T *depthData, int width, int height, int filterSize)
	{
		T *filterData = NULL;
		T *calcData = NULL;

		filterData = (T *)malloc(sizeof(T) * width * height);
		memset(filterData, 0, sizeof(T) * width * height);

		calcData = (T*)malloc(sizeof(T) * filterSize * filterSize);
		memset(calcData, 0, sizeof(T) * filterSize * filterSize);

		for (int h = filterSize / 2; h < height - filterSize / 2; h++)
		{
			for (int w = filterSize / 2; w < width - filterSize / 2; w++)
			{
				bool flag = true;
				for (int i = 0; i < filterSize; i++)
				{
					if (!flag)
						break;

					for (int j = 0; j < filterSize; j++)
					{
						if (depthData[(h + i - filterSize / 2) * width + w + j - filterSize / 2] < 1)
						{
							flag = false;
							break;
						}
						else
						{
							calcData[i* filterSize + j] = depthData[(h + i - filterSize / 2) * width + w + j - filterSize / 2];
						}
					}
				}

				if (flag)
				{
					T tmp = 0;

					for (int m = 0; m < filterSize * filterSize; m++)
					{
						for (int n = 0; n < m; n++)
						{
							if (calcData[n] < calcData[m])
							{
								tmp = calcData[n];
								calcData[n] = calcData[m];
								calcData[m] = tmp;
							}
						}
					}
					filterData[h * width + w] = calcData[filterSize * filterSize / 2];
				}
			}
		}

		int count = 0;

		for (int h = 0; h < height; h++)
		{
			for (int w = 0; w < width; w++)
			{
				if (filterData[h*width + w] > 1)
				{
					depthData[h*width + w] = filterData[h*width + w];
					count++;
				}
			}
		}
		free(filterData);
		filterData = NULL;

		free(calcData);
		calcData = NULL;

		return 0;
	}
	}
/*@} */
}

#endif
