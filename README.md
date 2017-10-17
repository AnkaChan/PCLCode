# PCLCode
1.本库包括源代码与exe文件

2.编译源代码需要最新的PCL库（1.8.1）与CMake（3.9.3）
	PCL 1.8.1： http://unanancyowen.com/en/pcl181/ 选择对应IDE版本的All in one installer并安装
	之后用Cmake建立对应的工程就可以编译
	
3.Bin/中装有编译好的exe程序。运行exe文件同样需要安装PCL1.8.1库，下载地址见上条
	只需要将要配准的文件放在./Bin/Data文件夹下。双击.bat就可以自动运行
	程序分为3步，每一步也可以单独运行：
	(1).m 转化为.pcd文件，对应m2obj.exe。该程序用法： m2obj inputPath outputPath
	(2).pcd点云化简，对应pcdSimplify.exe。 用法：pcdSimplify inputPath outputPath
	(3).pcd全局配准，对应LUM_Correct2.exe。用法：LUM_Correct2 SimplifiedDataPath OutputPath OriginalPcdDataPath
	
----------------------------
