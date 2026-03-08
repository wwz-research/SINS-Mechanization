/**
* @file     MatrixAndVect.cpp
* @brief    ���������������غ�����ʵ��
* @details  ���ļ�����������������������������λ��������Ӽ��ˡ�����ת�á����������Լ���������ɾ��������������ʵ�֣�\n
*           ���к������������������Ч�Խ��м�飬ȷ���������ȷ��
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/01
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/01, Weizhen Wang, Create
*           2025/11/02��Weizhen Wang�����Ӻ��� Normalize3(const int m, const double A[], double U[]) ���ڼ�����ά�����ĵ�λ������
*           2025/12/21��Weizhen Wang�����Ӻ��� SkewSymmetricMatrix(const int m, const double A[], double M[]) ���ڹ�����ά�����ķ��Գƾ���
*           2025/12/22��Weizhen Wang�����Ӻ��� double Norm3(const int m,const double scale, const double A[])���������ά������ģ��
*           2025/12/22��Weizhen Wang�����Ӻ��� EyeMatRowMajor(const int n, double I[])�������õ�λ����
*/

#include"MatrixAndVect.h"

/**
* @brief       ����һ�� n��n �ĵ�λ����һά���飬�������ȴ洢��
* @param[in]   n        const int       �����ά�ȣ�ӦΪ������
* @param[out]  I        double[]        �����һά���飬����Ӧ����Ϊ n*n���������ȴ洢��λ����
* @return      bool     ��λ���������Ƿ�ɹ���true ��ʾ�ɹ���false ��ʾʧ��
* @note        ��������������һ�� n��n �ĵ�λ������洢��ʽΪһά���飬���������ȣ�row-major����ʽ\n
*              ����Ԫ�ط��ʷ�ʽΪ��I[i*n + j]������ i Ϊ��������j Ϊ������\n
*              ������ά�� n ���Ϸ���n<=0�������������������Ϣ������ false
* @par History:
*              2025/12/22, Weizhen Wang, Create
* @internals   �����ڲ����ȼ�����ά�� n �ĺϷ��ԣ�Ȼ��ͨ��˫��ѭ����һά������и�ֵ\n
*              �� i == j ʱ����ӦԪ�ظ�ֵΪ 1.0������ֵΪ 0.0
*/
bool EyeMatRowMajor(const int n, double I[])
{
    // ������ά���Ƿ�Ϸ�
    if (n <= 0)
    {
        printf("Error dimension in EyeMatRowMajor!\n");
        return false;
    }

    // ���ɵ�λ���������ȵ�һά���飩
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i == j)
                I[i * n + j] = 1.0;
            else
                I[i * n + j] = 0.0;
        }
    }

    return true;
}


/**
* @brief       �������������ĵ��
* @param[in]   m        const int       ��һ������A��ά��
* @param[in]   n        const int       �ڶ�������B��ά��
* @param[in]   A        const double[]  ��һ����������
* @param[in]   B        const double[]  �ڶ�����������
* @return      double   �������������ά�Ȳ��Ϸ��򷵻�0.0
* @note        �������ڼ������������ĵ�����豣֤��������ά��Ϊ������ȣ��������������Ϣ������0.0\n
*              ������㹫ʽΪ��A��B = A[0]*B[0] + A[1]*B[1] + ... + A[m-1]*B[m-1]
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���ȼ������ά�ȵĺϷ��ԣ�m>0��n>0��m==n���������Ϸ������������Ϣ������0.0\n
*              �Ϸ�����£�ͨ��ѭ�������ӦԪ�س˻����ۼӺͣ��õ�������
*/
double VectDot(const int m, const int n, const double A[], const double B[])
{
    // �����ж����鳤���Ƿ�Ϸ���ƥ��
    if (m <= 0 || n <= 0 || m != n)
    {
        // ����򵥵ط���0.0��Ϊ����ֵ
        printf("Error dimension in VectDot!\n");
        return 0.0;
    }

    // �����������
    double DotResult = 0;
    for (int i = 0; i < m; i++)
    {
        DotResult = DotResult + A[i] * B[i];
    }

    return DotResult;
}

/**
* @brief       ����������ά�����Ĳ��
* @param[in]   m        const int       ��һ������A��ά��
* @param[in]   n        const int       �ڶ�������B��ά��
* @param[in]   A        const double[]  ��һ��������ά����
* @param[in]   B        const double[]  �ڶ���������ά����
* @param[out]  C        double[]        �洢����������ά����
* @return      bool     �����Ƿ�ɹ���true��ʾ�ɹ���false��ʾʧ��
* @note        �������ڼ���������ά�����Ĳ�����豣֤��������ά�Ⱦ�Ϊ3���������������Ϣ������false\n
*              ������㹫ʽΪ��C[0]=A[1]*B[2]-A[2]*B[1]��C[1]=A[2]*B[0]-A[0]*B[2]��C[2]=A[0]*B[1]-A[1]*B[0]
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���ȼ������ά���Ƿ��Ϊ3�������������������Ϣ������false\n
*              �Ϸ�����£����ݲ����ʽ���������洢��C�У�����true
*/
bool CrossDot(const int m, const int n, const double A[], const double B[], double C[])
{
    // �����ж�������ά���Ƿ���ȷ
    if (m != 3 || n != 3)
    {
        // �������ά�Ȳ���ȷ�����ش���ֵ
        printf("Error dimension in CrossDot!\n");
        return false;
    }

    // �����˽������
    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];

    return true;
}

/**
* @brief       ���Գƾ��󹹽�
* @param[in]   m      const int    ����������ά�ȣ�Ӧ�̶�Ϊ 3��
* @param[in]   A      const double ������ά��������ʽΪ A[0], A[1], A[2]��
* @param[out]  U      double       ���3*3����
* @return      bool   ���ص�λ���Ƿ�ɹ���true ��ʾ�ɹ���false ��ʾʧ�ܡ�
* @note        ��������A�ķ��Գƾ���\n
*              ������ά�� m ��Ϊ 3�������������� false ����ֹ���㡣
* @par History:
*              2025/12/21, Weizhen Wang, Create
* @internals   \n
*              
*/
bool SkewSymmetricMatrix(const int m, const double A[], double M[])
{
    // �����ж�������ά���Ƿ���ȷ
    if (m != 3)
    {
        // �������ά�Ȳ���ȷ�����ش���ֵ
        printf("Error dimension in CrossDot!\n");
        return false;
    }

    M[0] = 0.0;   M[1] = -A[2]; M[2] = A[1];
    M[3] = A[2];  M[4] = 0.0;   M[5] = -A[0];
    M[6] = -A[1]; M[7] = A[0];  M[8] = 0.0;

    return true;

}

/**
* @brief       ������ά�����ĵ�λ��������Normalize��
* @param[in]   m      const int    ����������ά�ȣ�Ӧ�̶�Ϊ 3��
* @param[in]   A      const double* ������ά��������ʽΪ A[0], A[1], A[2]��
* @param[out]  U      double*      �����λ���������� U = A / |A|��
* @return      bool   ���ص�λ���Ƿ�ɹ���true ��ʾ�ɹ���false ��ʾʧ�ܡ�
* @note        ���������ڶ���ά�������е�λ�������������������������һ��Ϊ����Ϊ 1 �ķ�������\n
*              ������ά�� m ��Ϊ 3�����������������쳣����ӽ��㣩ʱ�������������� false ����ֹ���㡣
* @par History:
*              2025/11/02, Weizhen Wang, Create
* @internals   �����ڲ����ȼ������ά���Ƿ�Ϊ 3��Ȼ�����ģ�� norm���������������ֱ���� norm\n
*              ��δ��������������飬��ͨ�� norm < 1e-12 ��ֵ�����һ��ʧ�ܡ�
*/
bool Normalize3(const int m, const double A[], double U[])
{
    // ���ά��
    if (m != 3)
    {
        printf("Error dimension in Normalize3!\n");
        return false;
    }

    // ��������ģ��
    double norm = sqrt(A[0] * A[0] +
        A[1] * A[1] +
        A[2] * A[2]);

    // ����������
    //if (norm < 1e-12)
    //{
    //    printf("Error: zero vector cannot be normalized!\n");
    //    return false;
    //}

    // ��λ��
    U[0] = A[0] / norm;
    U[1] = A[1] / norm;
    U[2] = A[2] / norm;

    return true;
}

/**
* @brief       ������ά������ģ����Euclidean Norm��
* @param[in]   m      const int     ����������ά�ȣ�Ӧ�̶�Ϊ 3��
* @param[in]   scale  const double  �����ı�������
* @param[in]   A      const double* ������ά��������ʽΪ A[0], A[1], A[2]��
* @return      double ��������������ģ�� |A|��������ʧ���򷵻� -1.0��
* @note        ���������ڼ�����ά������ŷ�����ģ������ |A| = sqrt(x^2 + y^2 + z^2)\n
*              ������ά�� m ��Ϊ 3 ʱ�������������� -1.0 ����ֹ���㡣
* @par History:
*              2025/11/22, Weizhen Wang, Create
* @internals   �����ڲ����ȼ������ά���Ƿ�Ϊ 3��Ȼ��ֱ�Ӱ���ŷ����ö������ģ��\n
*              ��δ��������������ֵ�ȶ����и���Ҫ�󣬿��ڴ˴�������ֵ�жϡ�
*/
double Norm3(const int m,const double scale, const double A[])
{
    // ���ά��
    if (m != 3)
    {
        printf("Error dimension in Norm3!\n");
        return -1.0;
    }

    // ��������ģ��
    double norm = sqrt(A[0] * A[0] +
        A[1] * A[1] +
        A[2] * A[2]);

    return norm;
}



/**
* @brief       ʵ����������ļӷ�����
* @param[in]   m        const int       ���������
* @param[in]   n        const int       ���������
* @param[in]   M1       const double[]  ��һ���������m��n��
* @param[in]   M2       const double[]  �ڶ����������m��n��
* @param[out]  M3       double[]        �洢�ӷ�����ľ���m��n��
* @return      bool     �����Ƿ�ɹ���true��ʾ�ɹ���false��ʾʧ��
* @note        �������ڼ�������ͬ�;���ĺͣ��豣֤����������������Ϊ�������������������Ϣ������false\n
*              ����ӷ�����Ϊ��M3[i][j] = M1[i][j] + M2[i][j]���������ȴ洢��
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���ȼ�����ά�ȵĺϷ��ԣ�m>0��n>0���������Ϸ������������Ϣ������false\n
*              �Ϸ�����£�ͨ��˫��ѭ����������Ԫ�أ������ӦԪ�صĺͲ��洢��M3�У�����true
*/
bool MatrixAddition(const int m, const int n, const double M1[], const double M2[], double M3[])
{
    // �����жϾ���������������Ƿ�Ϸ�
    if (m <= 0 || n <= 0)
    {
        // �������������С�ڵ���0�����ش���ֵ
        printf("Error dimension in MatrixAddition!\n");
        return false;
    }

    // ִ�о����������
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            int index = i * n + j;
            M3[index] = M1[index] + M2[index];
        }
    }

    return true;
}

/**
* @brief       ʵ����������ļ�������
* @param[in]   m        const int       ���������
* @param[in]   n        const int       ���������
* @param[in]   M1       const double[]  ��������m��n��
* @param[in]   M2       const double[]  ������m��n��
* @param[out]  M3       double[]        �洢��������ľ���m��n��
* @return      bool     �����Ƿ�ɹ���true��ʾ�ɹ���false��ʾʧ��
* @note        �������ڼ�������ͬ�;���Ĳ�豣֤����������������Ϊ�������������������Ϣ������false\n
*              �����������Ϊ��M3[i][j] = M1[i][j] - M2[i][j]���������ȴ洢��
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���ȼ�����ά�ȵĺϷ��ԣ�m>0��n>0���������Ϸ������������Ϣ������false\n
*              �Ϸ�����£�ͨ��˫��ѭ����������Ԫ�أ������ӦԪ�صĲ�洢��M3�У�����true
*/
bool MatrixSubtraction(const int m, const int n, const double M1[], const double M2[], double M3[])
{
    // �����жϾ���������������Ƿ�Ϸ�
    if (m <= 0 || n <= 0)
    {
        // �������������С�ڵ���0�����ش���ֵ
        printf("Error dimension in MatrixSubtraction!\n");
        return false;
    }
    // ִ�о����������
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            int index = i * n + j;
            M3[index] = M1[index] - M2[index];
        }
    }

    return true;
}

/**
* @brief       ʵ����������ĳ˷�����
* @param[in]   m1       const int        ��һ������M1������
* @param[in]   n1       const int        ��һ������M1���������ڶ�������M2��������
* @param[in]   m2       const int        �ڶ�������M2�������������n1��
* @param[in]   n2       const int        �ڶ�������M2������
* @param[in]   M1       const double[]  ��һ���������m1��n1��
* @param[in]   M2       const double[]  �ڶ����������m2��n2��
* @param[out]  M3       double[]        �洢�˷�����ľ���m1��n2��
* @return      bool     �����Ƿ�ɹ���true��ʾ�ɹ���false��ʾʧ��
* @note        �������ڼ�����������ĳ˻����豣֤����ά������M1��������M2����������ά��Ϊ�������������������Ϣ������false\n
*              ����˷�����Ϊ��M3[i][j] = ��(M1[i][k] * M2[k][j])��k��0��n1-1���������ȴ洢��
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���ȼ�����ά�ȵĺϷ��ԣ�m1>0��n1>0��m2>0��n2>0��n1==m2���������Ϸ������������Ϣ������false\n
*              �Ϸ�����£�ͨ������ѭ������˻�����Ԫ�أ��ȳ�ʼ��M3Ԫ��Ϊ0�����ۼӶ�Ӧ�˻�������true
*/
bool MatrixMultiply(const int m1, const int n1, const int m2, const int n2, const double M1[], const double M2[], double M3[])
{
    // �����жϾ�����˵Ĵ�С�Ƿ�Ϸ�
    if (m1 <= 0 || n1 <= 0 || m2 <= 0 || n2 <= 0 || n1 != m2)
    {
        printf("Error dimension in MatrixMultiply!\n");
        return false; // �����С���Ϸ����޷����о������
    }

    // ִ�о����������
    for (int i = 0; i < m1; i++)
    {
        for (int j = 0; j < n2; j++)
        {
            M3[i * n2 + j] = 0; // ��ʼ���������Ԫ��Ϊ0
            for (int k = 0; k < n1; k++)
            {
                M3[i * n2 + j] = M1[i * n1 + k] * M2[k * n2 + j] + M3[i * n2 + j]; // �������
            }
        }
    }

    return true;
}

/**
* @brief       ʵ�־����ת������
* @param[in]   m        const int       ԭ����M1������
* @param[in]   n        const int       ԭ����M1������
* @param[in]   M1       const double[]  ԭ����m��n��
* @param[out]  MT       double[]        �洢ת�ý���ľ���n��m��
* @return      bool     �����Ƿ�ɹ���true��ʾ�ɹ���false��ʾʧ��
* @note        �������ڼ�������ת�ã��豣֤����������������Ϊ�������������������Ϣ������false\n
*              ת�ù���Ϊ��MT[j][i] = M1[i][j]���������ȴ洢��ԭm��n����ת�ú�Ϊn��m��
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���ȼ�����ά�ȵĺϷ��ԣ�m>0��n>0���������Ϸ������������Ϣ������false\n
*              �Ϸ�����£�ͨ��˫��ѭ����ԭ����Ԫ��M1[i][j]��ֵ��ת�þ���MT[j][i]������true
*/
bool MatrixTranspose(const int m, const int n, const double M1[], double MT[])
{
    // �����жϾ���������������Ƿ�Ϸ�
    if (m <= 0 || n <= 0)
    {
        printf("Error dimension in MatrixTranspose!\n");
        return false; // �����С���Ϸ�
    }

    // ���о���ת������
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            MT[j * m + i] = M1[i * n + j]; // ��ת�ú��Ԫ�ش洢��MT
        }
    }

    return true;
}

/**
* @brief       ʵ�־�����������㣨����ȫѡ��Ԫ��˹-Լ������
* @param[in]   n        const int       ����Ľ���������
* @param[in]   a        const double[]  ԭ����n��n���������ȴ洢��
* @param[out]  b        double[]        �洢���������n��n���������ȴ洢��
* @return      bool     �����Ƿ�ɹ���true��ʾ�ɹ���false��ʾʧ��
* @note        �������ڼ���n�׷����������豣֤�������Ϊ�����Ҿ�������죬�������������Ϣ������false\n
*              ����ȫѡ��Ԫ��˹-Լ������ͨ���н������н���������Ԫ����߼����ȶ���
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���ȼ���������Ϸ��ԣ�n>0���������Ϸ������������Ϣ������false\n
*              �Ƚ�ԭ����a���Ƶ�b�У�ͨ��ȫѡ��Ԫ�ҵ�ÿ�����Ԫ��λ�ã������к��н������ٹ淶����Ԫ�к�������\n
*              ���ͨ���н������н�����ԭ��������Ԫ�ӽ�0��С��DBL_EPSILON�����ж�Ϊ������󣬷���false
*/
bool MatrixInv(const int n, const double a[], double b[])
{
    int i, j, k, l, u, v, is[200], js[200];
    double d, p;

    if (n <= 0)
    {
        printf("Error dimension in MatrixInv!\n");
        return false;
    }

    // ���������ֵ���������b�������b�������棬a���󲻱�
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            b[i * n + j] = a[i * n + j];
        }
    }

    for (k = 0; k < n; k++)
    {
        d = 0.0;
        for (i = k; i < n; i++)   // �������½Ƿ�������Ԫ�ص�λ�� 
        {
            for (j = k; j < n; j++)
            {
                l = n * i + j;
                p = fabs(b[l]);
                if (p > d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }

        if (d < DBL_EPSILON)   // ��Ԫ�ؽӽ���0�����󲻿���
        {
            printf("Divided by 0 in MatrixInv!\n");
            return false;
        }

        if (is[k] != k)  // ����Ԫ�����ڵ��������½Ƿ�������н��е���
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = is[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        if (js[k] != k)  // ����Ԫ�����ڵ��������½Ƿ�������н��е���
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = i * n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        l = k * n + k;
        b[l] = 1.0 / b[l];  // �����б任
        for (j = 0; j < n; j++)
        {
            if (j != k)
            {
                u = k * n + j;
                b[u] = b[u] * b[l];
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                for (j = 0; j < n; j++)
                {
                    if (j != k)
                    {
                        u = i * n + j;
                        b[u] = b[u] - b[i * n + k] * b[k * n + j];
                    }
                }
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                u = i * n + k;
                b[u] = -b[u] * b[l];
            }
        }
    }

    for (k = n - 1; k >= 0; k--)  // ����������е������»ָ�
    {
        if (js[k] != k)
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = js[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = is[k] + i * n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }

    return true;
}



/**
* @brief       ɾ��������ָ����һ�к�һ��
* @param[in]   m        const int  ԭ���������
* @param[in]   n        const int  ԭ���������
* @param[in]   m1       int        Ҫɾ������������1-based��
* @param[in]   n1       int        Ҫɾ������������1-based��
* @param[in,out] M      double[]   ����ԭ�������ɾ�����к�ľ��󣨰������ȴ洢��
* @note        ��������ɾ��������ָ����һ�к�һ�У��Ƚ�ָ���к��е�Ԫ����0���ٽ���0Ԫ��ǰ�ƣ�ʣ��λ�ò�0\n
*              �����������Ϊ1-based���ڲ�ת��Ϊ0-based����
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���Ƚ�1-based��������m1��������n1ת��Ϊ0-based����1��\n
*              ��ָ���е�����Ԫ�غ�ָ���е�����Ԫ����0��Ȼ��������󣬽���0Ԫ������ǰ�ƣ����ʣ��λ�ò�0
*/
void deleteRowAndColumn(const int m, const int n, int m1, int n1, double M[])
{
    m1 = m1 - 1;
    n1 = n1 - 1;
    // �����m1��Ԫ������
    for (int j = 0; j < n; j++) {
        M[m1 * n + j] = 0;
    }
    // �����n1��Ԫ������
    for (int i = 0; i < m; i++) {
        M[i * n + n1] = 0;
    }
    int count = 0; // ��ʼ��һ����������������¼����Ԫ�صĸ���
    int length = m * n;
    // �������飬�ѷ���Ԫ��Ų�����鿿ǰλ��
    for (int i = 0; i < length; i++)
    {
        if (M[i] != 0)
        {
            M[count] = M[i]; // ������Ԫ���Ƶ������ǰ��
            count++;
        }
    }
    // ��ʣ���λ��ȫ�����Ϊ0
    while (count < length)
    {
        M[count++] = 0;
    }
}

/**
* @brief       ɾ��������ָ����һ��
* @param[in]   rows        const int     ԭ���������������ȣ�
* @param[in]   rowToDelete const int     Ҫɾ������������1-based��
* @param[in,out] vector double[]         ����ԭ���������ɾ��ָ���к������
* @note        ��������ɾ��������ָ����һ�У��Ƚ�ָ���е�Ԫ����0���ٽ���0Ԫ��ǰ�ƣ�ʣ��λ�ò�0\n
*              ������������Ϊ1-based���ڲ�ת��Ϊ0-based����
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   �����ҵ�1-based��Ӧ��0-based������rowToDelete-1��������λ��Ԫ����0\n
*              ��������������0Ԫ������ǰ�ƣ����ʣ��λ�ò�0
*/
void deleteRow(const int rows, const int rowToDelete, double vector[])
{
    // ����ɾ�����е�����Ԫ������
    for (int i = 0; i < rows; i++)
    {
        if (i == (rowToDelete - 1))
        {
            vector[i] = 0;
            break;
        }
    }
    int count = 0; // ��ʼ��һ����������������¼����Ԫ�صĸ���
    int length = rows;
    // �������飬�ѷ���Ԫ��Ų�����鿿ǰλ��
    for (int i = 0; i < length; i++)
    {
        if (vector[i] != 0)
        {
            vector[count] = vector[i]; // ������Ԫ���Ƶ������ǰ��
            count++;
        }
    }
    // ��ʣ���λ��ȫ�����Ϊ0
    while (count < length)
    {
        vector[count++] = 0;
    }
}


/**
* @brief       �������Ĵ洢�ṹ
* @param[in]   a        const int        �¾��������
* @param[in]   b        const int        �¾��������
* @param[in]   m        const int        ԭ���������
* @param[in]   n        const int        ԭ���������
* @param[in,out] A      double[]   ����ԭ����m��n������������ľ���a��b���������ȴ洢��
* @note        �������ڽ�ԭ��������Ϊa��b���¾�����ȡԭ������ǰa��ǰb�е�Ԫ�أ�����λ�ò�0\n
*              ��ȷ���ڴ����ɹ����������������Ϣ
* @par History:
*              2025/11/01, Weizhen Wang, Create
* @internals   ���ȶ�̬����һ��a��b����ʱ����temp��������ʧ�������������Ϣ������\n
*              ��ԭ������i��j�У�i<a,j<b����Ԫ�ظ��Ƶ�temp�У��ٽ�temp�е�Ԫ�ظ��ƻ�A��ǰa��b��λ��\n
*              ���A��ʣ��λ�ã���a��b��m��n-1����0���ͷ���ʱ�����ڴ�
*/
void restructureMatrix(const int a, const int b, const int m, const int n, double A[])
{
    double* temp = (double*)malloc(a * b * sizeof(double)); // ����һ����ʱ�������洢�ع��������
    if (temp == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        return;
    }

    for (int i = 0; i < a; ++i) {
        for (int j = 0; j < b; ++j) {
            temp[i * b + j] = A[i * n + j]; // ��A�п���ʵ�ʾ�������ݵ�temp����
        }
    }

    for (int i = 0; i < a * b; ++i) {
        A[i] = temp[i]; // ���ع�������ݿ�����A����
    }

    // ����A����ʣ��Ĳ���Ϊ0
    for (int i = a * b; i < m * n; ++i) {
        A[i] = 0.0;
    }

    free(temp); // �ͷ���ʱ����
}