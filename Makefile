# author - Nazmul Alam (fadedreamz) - naz@atlantsembedded.com
# very basic Makefile
# date - July, 2017

TEST=test
TARGET=st1276
TEST_TARGET=test
TARGET_OBJS=st1276.o
TEST_OBJS=main.o

# change the following library if you want to use wiringPi
# we used a wrapper for it because wiringPi didn't worked well for the SPI
LDFLAGS+=-laePi
CFLAGS+=-I/usr/local/include/

st1276.o : st1276.c
	$(CC) -fPIC $(CFLAGS) -c st1276.c

$(TARGET) : $(TARGET_OBJS)
	$(CC) --shared $(TARGET_OBJS) -o lib$(TARGET).so $(LDFLAGS)

all: $(TEST_OBJS) $(TARGET)
	$(CC) $(TEST_OBJS) -o $(TEST_TARGET) $(LDFLAGS) -L. -l$(TARGET)

clean:
	- rm *.o lib$(TARGET).so $(TEST_TARGET) 2> /dev/null
