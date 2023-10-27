/**
* \copyright       Copyright (C) InvenSense 2021
* \brief           Calibration script running on Maestro board and calibarion process simulation
* \ingroup         robo_survey
* \file            GimmalControlConsole.cpp
* \author          D. Churikov
* \date            25.05.2021
*/

/* Description
 *  This simple application serves as an example of how to communicate with 
 *  the Maestro board, runs Calibration script, waits for script ending and
 *  runs it again to simulate calibration process on the robot.
 *
 *  This exampe based on Maestro Serial Example CWindows.
 *  https://www.pololu.com/docs/0J40/5.h.2
 *  
 *  See also cross platform example for the board here:
 *  https://www.pololu.com/docs/0J40/5.h.1
 * 
 *  The Maestro's serial commands are documented in the "Serial Interface"
 *  section of the Maestro user's guide:
 *  http://www.pololu.com/docs/0J40
 *
  *  REQUIREMENT: The Maestro's Serial Mode must be set to "USB Dual Port"
 *  or "USB Chained" for this program to work.
 */

#include <stdio.h>
#include <windows.h>
#include <string>
#include <iostream>

 /* Maestro cmd codes*/
const unsigned char MAESTRO_CMD_RUN_SUB = 0xA7;
const unsigned char MAESTRO_CMD_STOP_SCRIPT = 0xA4;
const unsigned char MAESTRO_CMD_GET_SCRIPT_STATUS = 0xAE;
const unsigned char MAESTRO_CMD_GET_ERROR = 0xA1;
const unsigned char MAESTRO_CMD_GET_POSITION = 0x90;


 /** Opens a handle to a serial port in Windows using CreateFile.
  * portName: The name of the port.
  *   Examples: "COM4", "USB#VID_1FFB&PID_0089&MI_04#6&3ad40bf600004#".
  * baudRate: The baud rate in bits per second.
  * Returns INVALID_HANDLE_VALUE if it fails.  Otherwise returns a handle to the port.
  */
HANDLE openPort(const char* portName, unsigned int baudRate)
{
	HANDLE port;
	DCB commState;
	BOOL success;
	COMMTIMEOUTS timeouts;

	/* Open the serial port. */
	port = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (port == INVALID_HANDLE_VALUE)
	{
		switch (GetLastError())
		{
		case ERROR_ACCESS_DENIED:
			fprintf(stderr, "Error: Access denied.  Try closing all other programs that are using the device.\n");
			break;
		case ERROR_FILE_NOT_FOUND:
			fprintf(stderr, "Error: Serial port not found.  "
				"Make sure that \"%s\" is the right port name.  "
				"Try closing all programs using the device and unplugging the "
				"device, or try rebooting.\n", portName);
			break;
		default:
			fprintf(stderr, "Error: Unable to open serial port.  Error code 0x%lx.\n", GetLastError());
			break;
		}
		return INVALID_HANDLE_VALUE;
	}

	/* Set the timeouts. */
	success = GetCommTimeouts(port, &timeouts);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to get comm timeouts.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}
	timeouts.ReadIntervalTimeout = 1000;
	timeouts.ReadTotalTimeoutConstant = 1000;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 1000;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	success = SetCommTimeouts(port, &timeouts);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to set comm timeouts.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	/* Set the baud rate. */
	success = GetCommState(port, &commState);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to get comm state.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}
	commState.BaudRate = baudRate;
	success = SetCommState(port, &commState);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to set comm state.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	/* Flush out any bytes received from the device earlier. */
	success = FlushFileBuffers(port);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to flush port buffers.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	return port;
}

/** Send Maestro command in port.
 * port: port handle
 * command: command array
 * sz_command: command array size
 * Returns 1 on success, 0 on failure.*/
BOOL maestroSendCmd(HANDLE port, const unsigned char *command, DWORD sz_command)
{
	DWORD bytesTransferred;

	// Send the command to the device.
	BOOL success = WriteFile(port, command, sz_command, &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to write command %x to serial port.  Error code 0x%lx.\n", command[0], GetLastError());
	}
	else if (sz_command != bytesTransferred)
	{
		success = false;
		fprintf(stderr, "Error: Expected to write %d bytes but only wrote %ld.\n", sz_command, bytesTransferred);
	}

	return success;
}

/** Read Maestro response from port.
 * port: port handle
 * command: command array
 * sz_command: command array size
 * Returns 1 on success, 0 on failure.*/
BOOL maestroReadResponse(HANDLE port, unsigned char* response, DWORD sz_response)
{
	DWORD bytesTransferred;
	BOOL success = false;

	// Read the response from the device.
	success = ReadFile(port, response, sz_response, &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to read response from serial port.  Error code 0x%lx.\n", GetLastError());
	}
	else if (sz_response != bytesTransferred)
	{
		success = false;
		fprintf(stderr, "Error: Expected to read %d bytes but only read %ld (timeout). "
			"Make sure the Maestro's serial mode is USB Dual Port or USB Chained.\n", sz_response, bytesTransferred);
	}

	return success;
}


/** Implements the Maestro's Restart Script at Subroutine command.
 * sub_number: sub number from 0 to 99
  * Returns 1 on success, 0 on failure.*/
BOOL maestroRunScriptSub(HANDLE port, unsigned char sub_number)
{
	unsigned char command[2] = { MAESTRO_CMD_RUN_SUB , sub_number };
	return maestroSendCmd(port, command, sizeof(command));
}

/** Implements the Maestro's Stop Script command.
   * Returns 1 on success, 0 on failure.*/
BOOL maestroStopScript(HANDLE port)
{
	unsigned char command[1] = { MAESTRO_CMD_STOP_SCRIPT };
	return maestroSendCmd(port, command, sizeof(command));
}

/** Implements the Maestro's Get Position serial command.
 * channel: Channel number from 0 to 23
 * position: A pointer to the returned position value (for a servo channel, the units are quarter-milliseconds)
 * Returns 1 on success, 0 on failure.*/
 BOOL maestroGetPosition(HANDLE port, unsigned char channel, unsigned short* position)
{
	DWORD bytesTransferred;

	unsigned char command[2] = { MAESTRO_CMD_GET_POSITION, channel };
	BOOL success = maestroSendCmd(port, command, sizeof(command));

	// Read the response from the device.
	unsigned char response[2];
	success = success ? maestroReadResponse(port, response, sizeof(response)) : false;

	// Convert the bytes received in to a position.
	*position = response[0] + 256 * response[1];

	return success;
}

 /** Implements the Maestro's Get Script Status serial command.
 * status: A pointer to the returned status value which is 0 or 1, to indicate whether the script is running (0) or stopped (1)
 * Returns 1 on success, 0 on failure.*/
 BOOL maestroGetScriptStatus(HANDLE port, unsigned char* status)
 {
	 DWORD bytesTransferred;

	 unsigned char command[1] = { MAESTRO_CMD_GET_SCRIPT_STATUS };
	 BOOL success = maestroSendCmd(port, command, sizeof(command));

	 // Read the response from the device.
	 unsigned char response[1];
	 success = success ? maestroReadResponse(port, response, sizeof(response)) : false;

	 // Convert the bytes received in to a position.
	 *status = response[0];

	 return success;
 }

 /** Implements the Maestro's Get Error serial command.
* error: A pointer to the returned error byte value which is 0 or 1, to indicate whether the script is running (0) or stopped (1)
* Returns 1 on success, 0 on failure.*/
 BOOL maestroGetError(HANDLE port, unsigned short* error)
 {
	 DWORD bytesTransferred;

	 unsigned char command[1] = { MAESTRO_CMD_GET_ERROR};
	 BOOL success = maestroSendCmd(port, command, sizeof(command));

	 // Read the response from the device.
	 unsigned char response[2];
	 success = success ? maestroReadResponse(port, response, sizeof(response)) : false;

	 // Convert the bytes received in to a position.
	 *error = response[0] + 0xff*response[1];

	 return success;
 }

int main(int argc, char* argv[])
{
	/* portName should match the name of the Maestro's Command Port
	 * as shown in your computer's Device Manager. */
	std::string portName = "\\\\.\\COM26";  // Each double slash in this source code represents one slash in the actual name.

	int baudRate = 9600;

	// Open the Maestro's serial port
	HANDLE port = openPort(portName.c_str(), baudRate);
	if (port == INVALID_HANDLE_VALUE) { return -1; }

	const int iteration_number = 8;

	for (int i = 1; i <= 8; i++)
	{
		BOOL success = maestroRunScriptSub(port, 0);
		
		std::cout << "Ireation " << i << ": ";
		
		unsigned char status = 1;
		unsigned short error = 0;
		do
		{
			Sleep(1000);
			success = maestroGetScriptStatus(port, &status);
			success = success && maestroGetError(port, &error);
			std::string status_str = std::string((error ? "e=" + std::to_string(error): "")) + (status ? "*" : ".");
			std::cout << status_str;
			
			// recomendation: read tag data and put them to calibralibration library here
		
		} while (success && (0 == status) && (0 == error));
		std::cout << "end of cycle" << std::endl;
		
		// recomendation: rotate robot here
	}
	
	// Close the serial port
	CloseHandle(port);

	// recomendation: estimate bias here

	return 0;
}


