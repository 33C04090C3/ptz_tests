/******************************************************************************/
/* PTZ TESTS                                                                  */
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <termios.h>

#include "ptz_tests.h"


const uint32_t get_fixed_config_address( const int fixed_config )
{
    switch( fixed_config )
    {
        case 1:
            return ADDRESS_3;
        case 2:
            return ADDRESS_4;
        case 5:
            return ADDRESS_1;
        case 6:
        case 7:
            return ADDRESS_2;
        default:
            return 0;
    }
}

// used for getting memory-mapped hardware register values
bool dev_mem_mmap_get_u32( uint32_t offset, uint32_t* result )
{
    bool     retVal    = false;
    uint8_t* pPtr      = NULL;
    int      fd_devmem = 0;

    // open /dev/mem
    fd_devmem = open( "/dev/mem", O_TRUNC | O_RDWR );
    if( fd_devmem < 0 )
    {
        printf( "dev_mem_mmap_get_u32() - error %d opening /dev/mem\n", errno );
        fd_devmem = 0;
        goto end;
    }

    // mmap a page of it
    pPtr = mmap( (void*)NULL, 
                  0x1000, 
                  PROT_READ | PROT_WRITE, 
                  MAP_SHARED, 
                  fd_devmem, 
                  offset & 0xFFFFF000 );
    if( pPtr == MAP_FAILED )
    {
        printf( "dev_mem_mmap_get_u32() - error %d mmapping offset %08X\n", offset & 0xFFFFF000, errno );
        goto end;
    }

    // retrieve a value
    *result = UINT32( pPtr + ( offset & 0xFFF ) );

    retVal = true;

end:
    if( pPtr != NULL && pPtr != MAP_FAILED )
    {
        munmap( pPtr, 0x1000 );
    }
    if( fd_devmem != 0 )
    {
        close( fd_devmem );
    }
    return retVal;
}

// used for setting memory-mapped hardware register values
bool dev_mem_mmap_put_u32( uint32_t offset, uint32_t result )
{

    bool     retVal    = false;
    uint8_t* pPtr      = NULL;
    int      fd_devmem = 0;

    // open /dev/mem
    fd_devmem = open( "/dev/mem", O_TRUNC | O_RDWR );
    if( fd_devmem < 0 )
    {
        printf( "dev_mem_mmap_put_u32() - error %d opening /dev/mem\n", errno );
        fd_devmem = 0;
        goto end;
    }

    // mmap a page of it
    pPtr = mmap( (void*)NULL, 
                  0x1000, 
                  PROT_READ | PROT_WRITE, 
                  MAP_SHARED, 
                  fd_devmem, 
                  offset & 0xFFFFF000 );
    if( pPtr == MAP_FAILED )
    {
        printf( "dev_mem_mmap_put_u32() - error %d mmapping offset %08X\n", offset & 0xFFFFF000, errno );
        goto end;
    }

    UINT32( pPtr + ( offset & 0xFFF ) ) = result;

    retVal = true;

end:
    if( pPtr != NULL && pPtr != MAP_FAILED )
    {
        munmap( pPtr, 0x1000 );
    }
    if( fd_devmem != 0 )
    {
        close( fd_devmem );
    }
    return retVal;
}

// write a buffer of data to the PTZ serial port
bool ptz_write_buffer( int ptz_fd, uint8_t* buffer, size_t len, int config_val )
{
    uint32_t tmp1 = 0;
    int cfg       = config_val;

    // at the moment we do not know exactly what this value is, but it determines which
    // address needs to have its low bit asserted
    uint32_t config_addr = get_fixed_config_address( cfg );

    if( config_addr != 0 )
    {
        if( dev_mem_mmap_get_u32( config_addr, &tmp1 ) == false )
        {
            printf( "ptz_write_buffer() config word %08X read fail\n", config_addr );
            return false;
        }
        tmp1 |= 1;
        if( dev_mem_mmap_put_u32( config_addr, tmp1 ) == false )
        {
            printf( "ptz_write_buffer() config word %08X write fail\n", config_addr );
            return false;
        }
    }

    // turn the CR bit RTS to 0 (not sure why, might be controlling the RS485 chip or something)
    // (may be active low)
    if( dev_mem_mmap_get_u32( HI3521_UART1_BASE + UART_CR, &tmp1 ) == false )
    {

        printf( "ptz_write_buffer() UART_CR read fail\n" );
        return false;
    }
    tmp1 &= 0xFFFFF7FF;
    if( dev_mem_mmap_put_u32( HI3521_UART1_BASE + UART_CR, tmp1 ) == false )
    {
        printf( "ptz_write_buffer() UART_CR write fail\n" );
        return false;
    }
    usleep(20000);

    // actually write out the data
    size_t i = 0;
    while( true )
    {
        size_t written = write( ptz_fd, buffer+i, len );
        if( (int)written < 0 )
        {
            printf( "ptz_write_buffer() data write failed: %d\n", errno );
            return false;
        }
        if( written == len )
        {
            break;
        }
        i += written;
        len -= written;
    }

    // wait for the serial port to finish sending it
    while(true)
    {
        // get flags
        if( dev_mem_mmap_get_u32( HI3521_UART1_BASE + UART_FR, &tmp1 ) == false )
        {
            printf( "ptz_write_buffer() UART_FR busy read fail\n" );
            return false;
        }
        tmp1 &= 0x88;
        // test for busy bit (bit 3) being clear, indicating the UART has finished sending
        if( tmp1 == 0x80 || tmp1 == 0 )
        {
            break;
        }
        usleep(0);
    }
    // turn the CR bit RTS back on (not sure why, might be controlling the RS485 chip or something)
    // (may be active low)
    if( dev_mem_mmap_get_u32( HI3521_UART1_BASE + UART_CR, &tmp1 ) == false )
    {
        printf( "ptz_write_buffer() UART_CR read fail\n" );
        return false;
    }
    tmp1 |= 0x800;
    if( dev_mem_mmap_put_u32( HI3521_UART1_BASE + UART_CR, tmp1 ) == false )
    {
        printf( "ptz_write_buffer() UART_CR write fail\n" );
        return false;
    }
    usleep(50000);

    return true;
}

bool ptz_pre_open(int config_val)
{

    uint32_t tmp1 = 0;

    switch( config_val )
    {
        case 1:
            dev_mem_mmap_put_u32( 0x200F004C, 0 );
            dev_mem_mmap_get_u32( 0x201A0400, &tmp1 );
            tmp1 |= 1;
            dev_mem_mmap_put_u32( 0x201A0400, tmp1 );
            dev_mem_mmap_put_u32( 0x200F0050, 1 );
            dev_mem_mmap_put_u32( 0x200F0058, 1);
            break;

        case 2:
            dev_mem_mmap_put_u32( 0x200F010C, 0 );
            dev_mem_mmap_get_u32( 0x201C0400, &tmp1 );
            tmp1 |= 1;
            dev_mem_mmap_put_u32( 0x201C0400, tmp1 );
            dev_mem_mmap_put_u32( 0x200F0110, 1 );
            dev_mem_mmap_put_u32( 0x200F0118, 1 );
            break;
        case 5:
            dev_mem_mmap_put_u32( 0x200F01A0, 0 );
            dev_mem_mmap_get_u32( 0x20210400, &tmp1 );
            tmp1 |= 0x40;
            dev_mem_mmap_put_u32( 0x20210400, tmp1 );
            dev_mem_mmap_put_u32( 0x200F01A4, 1 );
            dev_mem_mmap_put_u32( 0x200F01A8, 1 );
            break;
        case 6:
        case 7:
            dev_mem_mmap_put_u32( 0x200F0150, 0 );
            dev_mem_mmap_get_u32( 0x201E0400, &tmp1 );
            tmp1 |= 1;
            dev_mem_mmap_put_u32( 0x201E0400, tmp1 );
            dev_mem_mmap_put_u32( 0x200F0154, 1 );
            dev_mem_mmap_put_u32( 0x200F015C, 1 );
            break;
        default:
            printf( "[-] ptz_pre_open() - unrecognised config value %d\n", config_val );
            return false;
    }
    return true;
}

// for type 7
bool ptz_preconfig_improved(void)
{
    uint32_t tmp1 = 0;
    if( dev_mem_mmap_put( 0x200F0150, 0 ) == false )
    {
        printf( "[-] ptz_preconfig_improved() dev_mem_mmap_put(0x200F0150, 0) failed\n" );
        return false;
    }
    if( dev_mem_mmap_get( 0x201E0400, &tmp1 ) == false )
    {
        printf( "[-] ptz_preconfig_improved() dev_mem_mmap_get(0x200E0400) failed\n" );
        return false;
    }
    tmp1 |= 1;
    if( dev_mem_mmap_put( 0x201E0400, tmp1 ) == false )
    {
        printf( "[-] ptz_preconfig_improved() dev_mem_mmap_put(0x200E0400, %08X) failed\n", tmp1 );
        return false;
    }
    if( dev_mem_mmap_put( 0x200F0154, 1 ) == false )
    {
        printf( "[-] ptz_preconfig_improved() dev_mem_mmap_put(0x200F0154, 1) failed\n" );
        return false;
    }
    if( dev_mem_mmap_put( 0x200F015C, 1 ) == false )
    {
        printf( "[-] ptz_preconfig_improved() dev_mem_mmap_put(0x200F015C, 1) failed\n" );
        return false;
    }

    return true;
}

bool ptz_open( const char* devicename, int* fd_out )
{
    uint32_t tmp1 = 0;
    int fd        = 0;

    // these are the flags used by the DVR code
    fd = open( devicename, 0x902 );
    if( fd < 0 )
    {
        printf( "ptz_open() cannot open device %s - %u\n", devicename, errno );
        return false;
    }   

    if( dev_mem_mmap_get_u32( HI3521_UART1_BASE + UART_CR, &tmp1 ) == false )
    {
        printf( "ptz_open() UART_CR read fail\n" );
        close( fd );
        return false;
    }
    tmp1 |= 0x800;
    if( dev_mem_mmap_put_u32( HI3521_UART1_BASE + UART_CR, tmp1 ) == false )
    {
        printf( "ptz_open() UART_CR write fail\n" );
        close( fd );
        return false;
    }


    *fd_out = fd; 
    return true;
}

// This is the standard way to change the baud rate on Linux
bool set_baud_rate( int fd, speed_t baud_rate )
{
    struct termios t = {0};

    // set raw mode
    cfmakeraw(&t);
    // get the current settings
    if( tcgetattr( fd, &t ) < 0 )
    {
        printf( "tcgetattr() error %d", errno );
        return false;
    }
    // set the input speed
    if( cfsetispeed( &t, baud_rate ) < 0 )
    {
        printf( "cfsetispeed() error %d", errno );
        return false;
    }
    // set the output speed
    if( cfsetospeed( &t, baud_rate ) < 0 )
    {
        printf( "cfsetospeed() error %d", errno );
        return false;
    }
    // write the settings back
    if( tcsetattr( fd, TCSANOW, &t ) < 0 )
    {
        printf( "tcgetattr() error %d", errno );
        return true;
    }
    tcflush( fd, TCIOFLUSH );
    return true;
}

bool toggle_cts(int fd, bool on)
{
    int status = 0;
    if( ioctl( fd, TIOCMGET, &status ) < 0 )
    {
        printf( "[-] error getting TIOCMGET\n" );
        return false;    
    }

    if( on == true )
    {
        status |= TIOCM_CTS;
    }
    else
    {
        status &= ~TIOCM_CTS;
    }

    if( ioctl( fd, TIOCMSET, &status ) < 0 )
    {
        printf( "[-] error setting TIOCMGET\n" );
        return false;    
    }
}

bool toggle_rts(int fd, bool on)
{
    int status = 0;
    if( ioctl( fd, TIOCMGET, &status ) < 0 )
    {
        printf( "[-] error getting TIOCMGET\n" );
        return false;    
    }

    if( on == true )
    {
        status |= TIOCM_RTS;
    }
    else
    {
        status &= ~TIOCM_RTS;
    }

    if( ioctl( fd, TIOCMSET, &status ) < 0 )
    {
        printf( "[-] error setting TIOCMGET\n" );
        return false;    
    }
}


void print_usage( const char* name )
{
    printf( "Usage: %s [options]\n", name );
    printf( "Options:\n" );
    printf( "   -b : explicitly set baud rate to 9600\n" );
    printf( "   -h : print this help and don't do anything else\n" );
    printf( "   -p : try hardware preconfiguration\n" );
    printf( "   -1 : use configuration value 1\n" );
    printf( "   -2 : use configuration value 2\n" );
    printf( "   -5 : use configuration value 5\n" );
    printf( "   -6 : use configuration value 6" );
    printf( "   -7 : use configuration value 7\n" );
    printf( "   -i : use improved preconfiguration and other settings\n" );
}

int main( int argc, char* argv[] )
{
    int retVal = -1;
    int fd     = 0;
    // examples from https://www.commfront.com/pages/pelco-d-protocol-tutorial
    uint8_t left_at_high_speed[] = { 0xFF, 0x01, 0x00, 0x04, 0x3F, 0x00, 0x44 };
    uint8_t stop[]               = { 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01 };

    // This value controls what the write function does to set up the RS485 hardware
    // (the actual value is stored in the DVR configuration somewhere but not very accessibly)
    int config        = 0;
    bool do_baud_rate = false;
    bool preconfig    = false;
    bool improved     = false;
    printf( "[+] RS485 PELCO-D PTZ TESTER\n" );

    // check for command-line options
    for( int i = 1; i < argc; i++ )
    {
        // reject anything that's not a '-x' option
        if( ( strlen(argv[i]) != 2 ) || ( argv[i][0] != '-' ) )
        {
            continue;
        }
        switch(argv[i][1])
        {
            case 'h':
                print_usage(argv[0]);
                return 0;
            case 'b':
                do_baud_rate = true;
                break;
            case '1':
                config = 1;
                break;
            case '2':
                config = 2;
                break;
            case '5':
                config = 5;
                break;
            case '6':
                config = 6;
                break;
            case '7':
                config = 7;
                break;
            case 'i':
                improved = true;
                break;
            default:
                printf( "[-] unrecognised option '-%c'\n", argv[i][1] );
                return -1;
        }
    }

    if( improved == true )
    {
        if( ptz_preconfig_improved() == false )
        {
            goto end;
        }

        // actually do the work
        if( ptz_open( "/dev/ttyAMA1", &fd ) == false )
        {
            goto end;
        }

    }
    else
    {
        if( preconfig == true )
        {
            printf( "[+] Doing hardware preconfiguration...\n" );
            ptz_pre_open(config);
        }

        // actually do the work
        if( ptz_open( "/dev/ttyAMA1", &fd ) == false )
        {
            goto end;
        }
    }

    printf( "[+] Opened camera device...\n" );

    if( do_baud_rate == true )
    {
        if( set_baud_rate( fd, B9600 ) == false )
        {
            printf( "[-] Baud rate setting failed\n" );
        }
        else
        {
            printf( "[+] Baud rate set OK\n" );
        }
    }

    printf( "[+] Sending LEFT AT HIGH SPEED...\n" );

    if( ptz_write_buffer( fd, left_at_high_speed, 7, config ) == false )    
    {
        goto end;
    }
    sleep(1); 
    printf( "[+] Sending STOP ALL ACTIONS...\n" );
    if( ptz_write_buffer( fd, stop, 7, config ) == false )    
    {
        goto end;
    }

    retVal = 0;
end:
    if( fd > 0 )
    {
        close( fd );
    }
    return retVal;
}
