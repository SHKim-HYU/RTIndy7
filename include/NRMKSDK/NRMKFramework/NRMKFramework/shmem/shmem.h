#ifndef NRMKFRAMEWORK_SHMEM_H
#define NRMKFRAMEWORK_SHMEM_H

// std C headers.
#include <cstddef>

// platform-specific headers.
#if _MSC_VER
# define ON_WINDOWS		1
# include <windows.h>
#else
# define ON_WINDOWS		0
# include <unistd.h>
# include <fcntl.h>
# include <sys/mman.h>
#endif

// standard types.
#if ON_WINDOWS
# include "windows/stdint.h"
#else
# include <stdint.h>
#endif

#include "config.h"

// named shared memory
typedef struct shmem
{
	char*		name;
	uint8_t*	mapped;
#if ON_WINDOWS
	HANDLE		handle;
#else
	int			fd;
#endif
	size_t		size;
	bool		created;
} shmem_t;

struct shmem_header_t
{
	char		magic[4]; // "SMEM"
	char		pad[4];
	uint64_t	size;
};

shmem_t*	shmem_create( const char* name, size_t size );
shmem_t*	shmem_open( const char* name, size_t size );
shmem_t*	shmem_create_or_open( const char* name, size_t size );
void		shmem_close( shmem_t* shm );

void*		shmem_ptr( shmem_t* shm );

const char*	shmem_get_name( shmem_t* shm );
size_t		shmem_get_size( shmem_t* shm );
bool		shmem_was_created( shmem_t* shm );

#endif /* NRMKFRAMEWORK_SHMEM_H */
