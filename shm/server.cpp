#include "Shm.hpp"
#include "Fifo.hpp"

int main()
{
    Shm shm(pathname, projid, CREATER);
    shm.Attr();


    NamedFifo fifo(PATH, FILENAME);

    FileOper readerfile(PATH, FILENAME);
    readerfile.OpenForRead();

    char* mem = (char*)shm.VirtualAddr();
    while (true)
    {
        if (readerfile.Wait())
        {
            printf("%s\n", mem);
        }
        else
            break;
    }

    readerfile.Close();
    std::cout << "server end normal!" << std::endl;
    return 0;
}