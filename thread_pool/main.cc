#include "ProcessPool.hpp"

int main()
{
    // ������룬��һ���صñȽ����bug --- TODO
    // �������̳ض���
    ProcessPool pp(gdefaultnum);

    // �������̳�
    pp.Start();

    // �Զ��ɷ�����
    int cnt = 10;
    while (cnt--)
    {
        pp.Run();
        sleep(1);
    }

    // ���գ��������̳�
    pp.Stop();
    return 0;
}