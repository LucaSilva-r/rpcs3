// v406 USIO emulator

#include "stdafx.h"
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#endif

#include "usio.h"
#include "Input/pad_thread.h"
#include "Emu/Io/usio_config.h"
#include "Emu/IdManager.h"

#include "Emu/Cell/timers.hpp"
#include <queue>
#include <deque>
#include <mutex>
std::deque<int> g_taiko_queue[2][4];
std::mutex g_taiko_mutex;
LOG_CHANNEL(usio_log, "USIO");

namespace
{
constexpr u16 bngrw_bridge_port = 7766;

#ifdef _WIN32
using bngrw_sock_t = uptr;
constexpr bngrw_sock_t bngrw_invalid_sock = INVALID_SOCKET;
inline void bngrw_close_sock(bngrw_sock_t s) { ::closesocket(s); }
inline int bngrw_sock_errno() { return WSAGetLastError(); }
inline bool bngrw_would_block(int e) { return e == WSAEWOULDBLOCK; }
inline void bngrw_set_nonblocking(bngrw_sock_t s) { u_long nb = 1; ::ioctlsocket(s, FIONBIO, &nb); }
#else
using bngrw_sock_t = int;
constexpr bngrw_sock_t bngrw_invalid_sock = -1;
inline void bngrw_close_sock(bngrw_sock_t s) { ::close(s); }
inline int bngrw_sock_errno() { return errno; }
inline bool bngrw_would_block(int e) { return e == EAGAIN || e == EWOULDBLOCK; }
inline void bngrw_set_nonblocking(bngrw_sock_t s)
{
	const int flags = ::fcntl(s, F_GETFL, 0);
	::fcntl(s, F_SETFL, flags | O_NONBLOCK);
}
#endif

std::string bngrw_trim(std::string_view value)
{
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
	{
		value.remove_prefix(1);
	}

	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
	{
		value.remove_suffix(1);
	}

	return std::string(value);
}

std::vector<std::string> bngrw_split(std::string_view line)
{
	std::vector<std::string> parts;
	std::string cur;

	for (char ch : line)
	{
		if (std::isspace(static_cast<unsigned char>(ch)))
		{
			if (!cur.empty())
			{
				parts.push_back(cur);
				cur.clear();
			}
		}
		else
		{
			cur.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
		}
	}

	if (!cur.empty())
	{
		parts.push_back(cur);
	}

	return parts;
}

bool bngrw_hex_nibble(char ch, u8& out)
{
	if (ch >= '0' && ch <= '9')
	{
		out = static_cast<u8>(ch - '0');
		return true;
	}

	ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
	if (ch >= 'a' && ch <= 'f')
	{
		out = static_cast<u8>(ch - 'a' + 10);
		return true;
	}

	return false;
}

bool bngrw_parse_hex(std::string_view text, std::span<u8> out)
{
	std::vector<u8> nibbles;
	for (char ch : text)
	{
		u8 nibble = 0;
		if (bngrw_hex_nibble(ch, nibble))
		{
			nibbles.push_back(nibble);
		}
		else if (ch != ':' && ch != '-' && ch != '_' && !std::isspace(static_cast<unsigned char>(ch)))
		{
			return false;
		}
	}

	if (nibbles.size() != out.size() * 2)
	{
		return false;
	}

	for (usz i = 0; i < out.size(); i++)
	{
		out[i] = static_cast<u8>((nibbles[i * 2] << 4) | nibbles[i * 2 + 1]);
	}

	return true;
}
}

template <>
void fmt_class_string<usio_btn>::format(std::string& out, u64 arg)
{
	format_enum(out, arg, [](usio_btn value)
	{
		switch (value)
		{
		case usio_btn::test: return "Test";
		case usio_btn::coin: return "Coin";
		case usio_btn::service: return "Service";
		case usio_btn::enter: return "Enter/Start";
		case usio_btn::up: return "Up";
		case usio_btn::down: return "Down";
		case usio_btn::left: return "Left";
		case usio_btn::right: return "Right";
		case usio_btn::taiko_hit_side_left: return "Taiko Hit Side Left";
		case usio_btn::taiko_hit_side_right: return "Taiko Hit Side Right";
		case usio_btn::taiko_hit_center_left: return "Taiko Hit Center Left";
		case usio_btn::taiko_hit_center_right: return "Taiko Hit Center Right";
		case usio_btn::tekken_button1: return "Tekken Button 1";
		case usio_btn::tekken_button2: return "Tekken Button 2";
		case usio_btn::tekken_button3: return "Tekken Button 3";
		case usio_btn::tekken_button4: return "Tekken Button 4";
		case usio_btn::tekken_button5: return "Tekken Button 5";
		case usio_btn::count: return "Count";
		}

		return unknown;
	});
}

struct usio_memory
{
	std::vector<u8> backup_memory;

	usio_memory() = default;
	usio_memory(const usio_memory&) = delete;
	usio_memory& operator=(const usio_memory&) = delete;

	void init()
	{
		backup_memory.clear();
		backup_memory.resize(page_size * page_count);
	}

	static constexpr usz page_size = 0x10000;
	static constexpr usz page_count = 0x10;
};

usb_device_usio::usb_device_usio(const std::array<u8, 7>& location)
	: usb_device_emulated(location)
{
	// Initialize dependencies
	g_fxo->need<usio_memory>();

	device = UsbDescriptorNode(USB_DESCRIPTOR_DEVICE,
		UsbDeviceDescriptor{
			.bcdUSB             = 0x0110,
			.bDeviceClass       = 0xff,
			.bDeviceSubClass    = 0x00,
			.bDeviceProtocol    = 0xff,
			.bMaxPacketSize0    = 0x8,
			.idVendor           = 0x0b9a,
			.idProduct          = 0x0910,
			.bcdDevice          = 0x0910,
			.iManufacturer      = 0x01,
			.iProduct           = 0x02,
			.iSerialNumber      = 0x00,
			.bNumConfigurations = 0x01});

	auto& config0 = device.add_node(UsbDescriptorNode(USB_DESCRIPTOR_CONFIG,
		UsbDeviceConfiguration{
			.wTotalLength        = 39,
			.bNumInterfaces      = 0x01,
			.bConfigurationValue = 0x01,
			.iConfiguration      = 0x00,
			.bmAttributes        = 0xc0,
			.bMaxPower           = 0x32 // ??? 100ma
		}));

	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_INTERFACE,
		UsbDeviceInterface{
			.bInterfaceNumber   = 0x00,
			.bAlternateSetting  = 0x00,
			.bNumEndpoints      = 0x03,
			.bInterfaceClass    = 0x00,
			.bInterfaceSubClass = 0x00,
			.bInterfaceProtocol = 0x00,
			.iInterface         = 0x00}));

	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint{
			.bEndpointAddress = 0x01,
			.bmAttributes     = 0x02,
			.wMaxPacketSize   = 0x0040,
			.bInterval        = 0x00}));

	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint{
			.bEndpointAddress = 0x82,
			.bmAttributes     = 0x02,
			.wMaxPacketSize   = 0x0040,
			.bInterval        = 0x00}));

	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint{
			.bEndpointAddress = 0x83,
			.bmAttributes     = 0x03,
			.wMaxPacketSize   = 0x0008,
			.bInterval        = 16}));

	load_backup();
	bngrw_bridge_init();
}

usb_device_usio::~usb_device_usio()
{
	if (m_bngrw_client != ~uptr{0})
	{
		bngrw_close_sock(static_cast<bngrw_sock_t>(m_bngrw_client));
		m_bngrw_client = ~uptr{0};
	}
	if (m_bngrw_listen != ~uptr{0})
	{
		bngrw_close_sock(static_cast<bngrw_sock_t>(m_bngrw_listen));
		m_bngrw_listen = ~uptr{0};
	}

	save_backup();
}

std::shared_ptr<usb_device> usb_device_usio::make_instance(u32 controller_index, const std::array<u8, 7>& location)
{
	if (controller_index == 1)
	{
		return std::make_shared<usb_device_bngrw>(location);
	}

	return std::make_shared<usb_device_usio>(location);
}

u16 usb_device_usio::get_num_emu_devices()
{
	return 2;
}

void usb_device_usio::control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer)
{
	transfer->fake = true;

	// Control transfers are nearly instant
	//switch (bmRequestType)
	{
	//default:
		// Follow to default emulated handler
		usb_device_emulated::control_transfer(bmRequestType, bRequest, wValue, wIndex, wLength, buf_size, buf, transfer);
		//break;
	}
}

extern bool is_input_allowed();

void usb_device_usio::load_backup()
{
	usio_memory& memory = g_fxo->get<usio_memory>();
	memory.init();

	fs::file usio_backup_file;

	if (!usio_backup_file.open(usio_backup_path, fs::read))
	{
		usio_log.trace("Failed to load the USIO Backup file: %s", usio_backup_path);
		return;
	}

	const u64 file_size = memory.backup_memory.size();

	if (usio_backup_file.size() != file_size)
	{
		usio_log.trace("Invalid USIO Backup file detected: %s", usio_backup_path);
		return;
	}

	usio_backup_file.read(memory.backup_memory.data(), file_size);
}

void usb_device_usio::save_backup()
{
	if (!is_used)
		return;

	fs::file usio_backup_file;

	if (!usio_backup_file.open(usio_backup_path, fs::create + fs::write + fs::lock))
	{
		usio_log.error("Failed to save the USIO Backup file: %s", usio_backup_path);
		return;
	}

	const u64 file_size = g_fxo->get<usio_memory>().backup_memory.size();

	usio_backup_file.write(g_fxo->get<usio_memory>().backup_memory.data(), file_size);
	usio_backup_file.trunc(file_size);
}

void usb_device_usio::translate_input_taiko()
{
	std::lock_guard lock(pad::g_pad_mutex);
	const auto handler = pad::get_pad_thread();

	std::vector<u8> input_buf(0x60);
	le_t<u16> digital_input = 0;

	// 値を反転させるためのスイッチ（50⇔51）
	static bool valueStates[2][4] = {};

	const auto fire_hit = [&](u8* ptr, usz player, int lane)
	{
		if (!ptr)
			return;

		bool& state = valueStates[player][lane];
		u16 hit_val = state ? 51 : 50;
		state = !state;

		u16 analog_val = (hit_val << 15) / 100 + 1;
		le_t<u16> out = analog_val;
		std::memcpy(ptr, &out, sizeof(u16));
	};

	const auto translate_from_pad = [&](usz pad_num, usz player)
	{
		if (player >= 2 || pad_num >= g_cfg_usio.players.size())
			return;

		const usz offset = player * 8ULL;
		auto& status = m_io_status[0];

		if (const auto& pad = ::at32(handler->GetPads(), pad_num); (pad->m_port_status & CELL_PAD_STATUS_CONNECTED) && is_input_allowed())
		{
			const auto& cfg = ::at32(g_cfg_usio.players, pad_num);

			cfg->handle_input(pad, false, [&](usio_btn btn, pad_button, u16, bool pressed, bool&)
				{
					if (btn == usio_btn::test && player == 0)
					{
						if (pressed && !status.test_key_pressed)
							status.test_on = !status.test_on;
						status.test_key_pressed = pressed;
					}
					else if (btn == usio_btn::coin && player == 0)
					{
						if (pressed && !status.coin_key_pressed)
							status.coin_counter++;
						status.coin_key_pressed = pressed;
					}
					else if (btn == usio_btn::service && player == 0 && pressed)
						digital_input |= 0x4000;
					else if (btn == usio_btn::enter && player == 0 && pressed)
						digital_input |= 0x200;
					else if (btn == usio_btn::up && player == 0 && pressed)
						digital_input |= 0x2000;
					else if (btn == usio_btn::down && player == 0 && pressed)
						digital_input |= 0x1000;
				});
		}
		else
		{
			// 切断時のリセット処理
			std::lock_guard<std::mutex> lock(g_taiko_mutex);
			for (int i = 0; i < 4; ++i)
			{
				valueStates[player][i] = false;
				g_taiko_queue[player][i].clear();
			}
		}

		if (player == 0 && status.test_on)
			digital_input |= 0x80;

		// キューの消費処理
		for (int i = 0; i < 4; ++i)
		{
			std::lock_guard<std::mutex> lock(g_taiko_mutex);

			if (!g_taiko_queue[player][i].empty())
			{
				// usio_log.error を使うとログ画面で目立つ色（赤）で表示されます
				// usio_log.error("USIO: Pop Hit! Player: %d, Lane: %d, Remaining in Queue: %llu",
				//(int)player, i, (unsigned long long)g_taiko_queue[player][i].size());
				g_taiko_queue[player][i].pop_front();

				// ★修正ポイント: +34 を +32 に戻す
				fire_hit(input_buf.data() + 32 + offset + i * 2, player, i);
			}
		}
	};

	for (usz i = 0; i < g_cfg_usio.players.size(); i++)
		translate_from_pad(i, i);

	std::memcpy(input_buf.data(), &digital_input, sizeof(u16));
	std::memcpy(input_buf.data() + 16, &m_io_status[0].coin_counter, sizeof(u16));

	response = std::move(input_buf);
}

void usb_device_usio::translate_input_tekken()
{
	std::lock_guard lock(pad::g_pad_mutex);
	const auto handler = pad::get_pad_thread();

	std::vector<u8> input_buf(0x180);
	le_t<u64> digital_input[2]{};
	le_t<u16> digital_input_lm = 0;

	const auto translate_from_pad = [&](usz pad_number, usz player)
	{
		const usz shift = (player % 2) * 24ULL;
		auto& status = m_io_status[player / 2];
		auto& input = digital_input[player / 2];

		if (const auto& pad = ::at32(handler->GetPads(), pad_number); pad->is_connected() && !pad->is_copilot() && is_input_allowed())
		{
			const auto& cfg = ::at32(g_cfg_usio.players, pad_number);
			cfg->handle_input(pad, false, [&](usio_btn btn, pad_button /*pad_btn*/, u16 /*value*/, bool pressed, bool& /*abort*/)
			{
				switch (btn)
				{
				case usio_btn::test:
					if (player % 2 != 0)
						break;
					if (pressed && !status.test_key_pressed) // Solve the need to hold the Test button
						status.test_on = !status.test_on;
					status.test_key_pressed = pressed;
					break;
				case usio_btn::coin:
					if (player % 2 != 0)
						break;
					if (pressed && !status.coin_key_pressed) // Ensure only one coin is inserted each time the Coin button is pressed
						status.coin_counter++;
					status.coin_key_pressed = pressed;
					break;
				case usio_btn::service:
					if (player % 2 == 0 && pressed)
						input |= 0x4000;
					break;
				case usio_btn::enter:
					if (pressed)
					{
						input |= 0x800000ULL << shift;
						if (player == 0)
							digital_input_lm |= 0x800;
					}
					break;
				case usio_btn::up:
					if (pressed)
					{
						input |= 0x200000ULL << shift;
						if (player == 0)
							digital_input_lm |= 0x200;
					}
					break;
				case usio_btn::down:
					if (pressed)
					{
						input |= 0x100000ULL << shift;
						if (player == 0)
							digital_input_lm |= 0x400;
					}
					break;
				case usio_btn::left:
					if (pressed)
					{
						input |= 0x80000ULL << shift;
						if (player == 0)
							digital_input_lm |= 0x2000;
					}
					break;
				case usio_btn::right:
					if (pressed)
					{
						input |= 0x40000ULL << shift;
						if (player == 0)
							digital_input_lm |= 0x4000;
					}
					break;
				case usio_btn::tekken_button1:
					if (pressed)
					{
						input |= 0x20000ULL << shift;
						if (player == 0)
							digital_input_lm |= 0x100;
					}
					break;
				case usio_btn::tekken_button2:
					if (pressed)
						input |= 0x10000ULL << shift;
					break;
				case usio_btn::tekken_button3:
					if (pressed)
						input |= 0x40000000ULL << shift;
					break;
				case usio_btn::tekken_button4:
					if (pressed)
						input |= 0x20000000ULL << shift;
					break;
				case usio_btn::tekken_button5:
					if (pressed)
						input |= 0x80000000ULL << shift;
					break;
				default:
					break;
				}
			});
		}

		if (player % 2 == 0 && status.test_on)
		{
			input |= 0x80;
			if (player == 0)
				digital_input_lm |= 0x1000;
		}
	};

	for (usz i = 0; i < g_cfg_usio.players.size(); i++)
		translate_from_pad(i, i);

	for (usz i = 0; i < 2; i++)
	{
		std::memcpy(input_buf.data() - i * 0x80 + 0x100, &digital_input[i], sizeof(u64));
		std::memcpy(input_buf.data() - i * 0x80 + 0x100 + 0x10, &m_io_status[i].coin_counter, sizeof(u16));
	}

	std::memcpy(input_buf.data(), &digital_input_lm, sizeof(u16));

	input_buf[2] = 0b00010000; // DIP switches, 8 in total

	response = std::move(input_buf);
}

void usb_device_usio::usio_write(u8 channel, u16 reg, std::vector<u8>& data)
{
	const auto get_u16 = [&](std::string_view usio_func) -> u16
	{
		if (data.size() != 2)
		{
			usio_log.error("data.size() is %d, expected 2 for get_u16 in %s", data.size(), usio_func);
		}
		return *reinterpret_cast<const le_t<u16>*>(data.data());
	};

	if (channel == 0)
	{
		switch (reg)
		{
		case 0x0002:
		{
			usio_log.trace("SetSystemError: 0x%04X", get_u16("SetSystemError"));
			break;
		}
		case 0x000A:
		{
			if (get_u16("ClearSram") == 0x6666)
			    usio_log.trace("ClearSram");
			break;
		}
		case 0x0028:
		{
			usio_log.trace("SetExpansionMode: 0x%04X", get_u16("SetExpansionMode"));
			break;
		}
		case 0x0048:
		case 0x0058:
		case 0x0068:
		case 0x0078:
		{
			usio_log.trace("SetHopperRequest(Hopper: %d, Request: 0x%04X)", (reg - 0x48) / 0x10, get_u16("SetHopperRequest"));
			break;
		}
		case 0x004A:
		case 0x005A:
		case 0x006A:
		case 0x007A:
		{
			usio_log.trace("SetHopperRequest(Hopper: %d, Limit: 0x%04X)", (reg - 0x4A) / 0x10, get_u16("SetHopperLimit"));
			break;
		}
		case 0x7000:
		{
			usio_log.notice("BNGRW-USIO control write: %s", fmt::buf_to_hexstring(data.data(), data.size()));
			break;
		}
		case 0x7400:
		{
			usio_log.notice("BNGRW-USIO write: %s", fmt::buf_to_hexstring(data.data(), data.size()));
			bngrw_feed_bytes(data.data(), ::size32(data));
			break;
		}
		default:
		{
			usio_log.trace("Unhandled channel 0 register write(reg: 0x%04X, size: 0x%04X, data: %s)", reg, data.size(), fmt::buf_to_hexstring(data.data(), data.size()));
			break;
		}
		}
	}
	else if (channel >= 2)
	{
		const u8 page = channel - 2;
		usio_log.trace("Usio write of sram(page: 0x%02X, addr: 0x%04X, size: 0x%04X, data: %s)", page, reg, data.size(), fmt::buf_to_hexstring(data.data(), data.size()));
		auto& memory = g_fxo->get<usio_memory>().backup_memory;
		const usz addr_end = reg + data.size();
		if (data.size() > 0 && page < usio_memory::page_count && addr_end <= usio_memory::page_size)
			std::memcpy(&memory[usio_memory::page_size * page + reg], data.data(), data.size());
		else
			usio_log.error("Usio sram invalid write operation(page: 0x%02X, addr: 0x%04X, size: 0x%04X, data: %s)", page, reg, data.size(), fmt::buf_to_hexstring(data.data(), data.size()));
	}
	else
	{
		// Channel 1 is the endpoint for firmware update.
		// We are not using any firmware since this is emulation.
		usio_log.trace("Unsupported write operation(channel: 0x%02X, addr: 0x%04X, size: 0x%04X, data: %s)", channel, reg, data.size(), fmt::buf_to_hexstring(data.data(), data.size()));
	}
}

void usb_device_usio::usio_read(u8 channel, u16 reg, u16 size)
{
	if (channel == 0)
	{
		switch (reg)
		{
		case 0x0000:
		{
			// Get Buffer, rarely gives a reply on real HW
			// First U16 seems to be a timestamp of sort
			// Purpose seems related to connectivity check
			response = {0x7E, 0xE4, 0x00, 0x00, 0x74, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
			break;
		}
		case 0x0080:
		{
			response = {0x02, 0x03, 0x06, 0x00, 0xFF, 0x0F, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x10, 0x00};
			// Card reader UART status. Byte 2 is the pending RX count used by
			// the game before it reads from 0x7000.
			response[2] = static_cast<u8>(std::min<usz>(m_bngrw_response.size(), 0xff));
			usio_log.notice("BNGRW-USIO status pending=%u", response[2]);
			break;
		}
		case 0x7000:
		{
			const u32 read_size = std::min<u32>(size, ::size32(m_bngrw_response));
			for (u32 i = 0; i < read_size; i++)
			{
				response.push_back(m_bngrw_response.front());
				m_bngrw_response.pop_front();
			}
			if (read_size != 0)
			{
				usio_log.notice("BNGRW-USIO read: %s", fmt::buf_to_hexstring(response.data(), response.size()));
			}
			break;
		}
		case 0x1000:
		{
			// Often called, gets input from usio for Tekken
			translate_input_tekken();
			break;
		}
		case 0x1080:
		{
			// Often called, gets input from usio for Taiko
			translate_input_taiko();
			break;
		}
		case 0x1800:
		case 0x1880:
		{
			// Seems to contain a few extra bytes of info in addition to the firmware string
			// Firmware
			// "NBGI.;USIO01;Ver1.00;JPN,Multipurpose with PPG."
			constexpr std::array<u8, 0x180> info {0x4E, 0x42, 0x47, 0x49, 0x2E, 0x3B, 0x55, 0x53, 0x49, 0x4F, 0x30, 0x31, 0x3B, 0x56, 0x65, 0x72, 0x31, 0x2E, 0x30, 0x30, 0x3B, 0x4A, 0x50, 0x4E, 0x2C, 0x4D, 0x75, 0x6C, 0x74, 0x69, 0x70, 0x75, 0x72, 0x70, 0x6F, 0x73, 0x65, 0x20, 0x77, 0x69, 0x74, 0x68, 0x20, 0x50, 0x50, 0x47, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4E, 0x42, 0x47, 0x49, 0x31, 0x3B, 0x55, 0x53, 0x49, 0x4F, 0x30, 0x31, 0x3B, 0x56, 0x65, 0x72, 0x31, 0x2E, 0x30, 0x30, 0x3B, 0x4A, 0x50, 0x4E, 0x2C, 0x4D, 0x75, 0x6C, 0x74, 0x69, 0x70, 0x75, 0x72, 0x70, 0x6F, 0x73, 0x65, 0x20, 0x77, 0x69, 0x74, 0x68, 0x20, 0x50, 0x50, 0x47, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x13, 0x00, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x02, 0x00, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x75, 0x6C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4E, 0x42, 0x47, 0x49, 0x32, 0x3B, 0x55, 0x53, 0x49, 0x4F, 0x30, 0x31, 0x3B, 0x56, 0x65, 0x72, 0x31, 0x2E, 0x30, 0x30, 0x3B, 0x4A, 0x50, 0x4E, 0x2C, 0x4D, 0x75, 0x6C, 0x74, 0x69, 0x70, 0x75, 0x72, 0x70, 0x6F, 0x73, 0x65, 0x20, 0x77, 0x69, 0x74, 0x68, 0x20, 0x50, 0x50, 0x47, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x13, 0x00, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x02, 0x00, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x75, 0x6C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
			response = {info.begin() + (reg - 0x1800), info.end()};
			break;
		}
		default:
		{
			usio_log.trace("Unhandled channel 0 register read(reg: 0x%04X, size: 0x%04X)", reg, size);
			break;
		}
		}
	}
	else if (channel >= 2)
	{
		const u8 page = channel - 2;
		usio_log.trace("Usio read of sram(page: 0x%02X, addr: 0x%04X, size: 0x%04X)", page, reg, size);
		auto& memory = g_fxo->get<usio_memory>().backup_memory;
		const usz addr_end = reg + size;
		if (size > 0 && page < usio_memory::page_count && addr_end <= usio_memory::page_size)
			response.insert(response.end(), memory.begin() + (usio_memory::page_size * page + reg), memory.begin() + (usio_memory::page_size * page + addr_end));
		else
			usio_log.error("Usio sram invalid read operation(page: 0x%02X, addr: 0x%04X, size: 0x%04X)", page, reg, size);
	}
	else
	{
		// Channel 1 is the endpoint for firmware update.
		// We are not using any firmware since this is emulation.
		usio_log.trace("Unsupported read operation(channel: 0x%02X, addr: 0x%04X, size: 0x%04X)", channel, reg, size);
	}

	response.resize(size); // Always resize the response vector to the given size
}

void usb_device_usio::usio_init(u8 channel, u16 reg, u16 size)
{
	if (channel == 0)
	{
		switch (reg)
		{
		case 0x0008:
		{
			usio_log.trace("USIO Reset");
			break;
		}
		case 0x000A:
		{
			usio_log.trace("USIO ClearSram");
			g_fxo->get<usio_memory>().init();
			break;
		}
		default:
		{
			usio_log.trace("Unhandled channel 0 register init(reg: 0x%04X, size: 0x%04X)", reg, size);
			break;
		}
		}
	}
	else
	{
		usio_log.trace("Unsupported init operation(channel: 0x%02X, addr: 0x%04X, size: 0x%04X)", channel, reg, size);
	}
}

void usb_device_usio::bngrw_send_ack()
{
	static constexpr std::array<u8, 6> ack{0x00, 0x00, 0xff, 0x00, 0xff, 0x00};
	m_bngrw_response.insert(m_bngrw_response.end(), ack.begin(), ack.end());
}

void usb_device_usio::bngrw_send_response(u8 cmd, std::span<const u8> payload)
{
	std::vector<u8> frame;
	const u8 len = static_cast<u8>(2 + payload.size());
	frame.reserve(static_cast<usz>(len) + 7);
	frame.insert(frame.end(), {0x00, 0x00, 0xff, len, static_cast<u8>(~len + 1), 0xd5, static_cast<u8>(cmd + 1)});
	frame.insert(frame.end(), payload.begin(), payload.end());

	u8 checksum = 0xff;
	for (u8 i = 0; i < len; i++)
	{
		checksum += frame[5 + i];
	}

	frame.push_back(static_cast<u8>(~checksum));
	frame.push_back(0x00);
	m_bngrw_response.insert(m_bngrw_response.end(), frame.begin(), frame.end());
	usio_log.notice("BNGRW-USIO response cmd=0x%02x data=%s", cmd, fmt::buf_to_hexstring(payload.data(), payload.size()));
}

void usb_device_usio::bngrw_send_response(u8 cmd, std::initializer_list<u8> payload)
{
	bngrw_send_response(cmd, std::span<const u8>(payload.begin(), payload.size()));
}

void usb_device_usio::bngrw_send_simple_response()
{
	bngrw_send_response(m_bngrw_request[6]);
}

void usb_device_usio::bngrw_cmd_gpio(std::span<const u8> data)
{
	if (data.size() >= 2)
	{
		if (data[0] == 0x01)
		{
			usio_log.notice("BNGRW-USIO LED: 0x%02x", data[1]);
			bngrw_bridge_send_event(fmt::format("event led 0x%02x", data[1]));
		}
		else if (data[0] == 0x08)
		{
			usio_log.notice("BNGRW-USIO BEEP: 0x%02x", data[1]);
			bngrw_bridge_send_event(fmt::format("event beep 0x%02x", data[1]));
		}
		else
		{
			bngrw_bridge_send_event(fmt::format("event gpio port=0x%02x value=0x%02x", data[0], data[1]));
		}
	}

	bngrw_send_simple_response();
}

void usb_device_usio::bngrw_cmd_rf_field(std::span<const u8> data)
{
	const bool off = data.size() >= 2 && data[0] == 0x01 && data[1] == 0x00;
	usio_log.notice("BNGRW-USIO RF field: %s", off ? "off" : "on");
	bngrw_bridge_send_event(off ? "event rf off" : "event rf on");
	bngrw_send_simple_response();
}

void usb_device_usio::bngrw_cmd_poll_card()
{
	bngrw_bridge_poll();

	switch (m_bngrw_card.type)
	{
	case bngrw_card_type::mifare:
	{
		std::array<u8, 10> card{
			0x01, 0x01, 0x00, 0x04,
			0x08, 0x04,
			m_bngrw_card.uid[0], m_bngrw_card.uid[1], m_bngrw_card.uid[2], m_bngrw_card.uid[3]
		};
		bngrw_send_response(m_bngrw_request[6], card);
		break;
	}
	case bngrw_card_type::felica:
	{
		std::array<u8, 22> card{
			0x01, 0x01, 0x14, 0x01,
			m_bngrw_card.idm[0], m_bngrw_card.idm[1], m_bngrw_card.idm[2], m_bngrw_card.idm[3],
			m_bngrw_card.idm[4], m_bngrw_card.idm[5], m_bngrw_card.idm[6], m_bngrw_card.idm[7],
			m_bngrw_card.pmm[0], m_bngrw_card.pmm[1], m_bngrw_card.pmm[2], m_bngrw_card.pmm[3],
			m_bngrw_card.pmm[4], m_bngrw_card.pmm[5], m_bngrw_card.pmm[6], m_bngrw_card.pmm[7],
			m_bngrw_card.system_code[0], m_bngrw_card.system_code[1]
		};
		bngrw_send_response(m_bngrw_request[6], card);
		break;
	}
	case bngrw_card_type::none:
	default:
		bngrw_send_response(m_bngrw_request[6], {0x00, 0x00, 0x00});
		break;
	}
}

void usb_device_usio::bngrw_cmd_mifare(std::span<const u8> data)
{
	if (data.size() < 2)
	{
		bngrw_send_ack();
		return;
	}

	switch (data[1])
	{
	case 0x60:
	case 0x61:
		bngrw_bridge_send_event(fmt::format("event mifare auth key=%c block=%u", data[1] == 0x60 ? 'A' : 'B', data.size() >= 3 ? data[2] : 0));
		bngrw_send_response(m_bngrw_request[6], {0x01});
		break;
	case 0x30:
		bngrw_bridge_send_event(fmt::format("event mifare read block=%u", data.size() >= 3 ? data[2] : 0));
		bngrw_send_response(m_bngrw_request[6], {0x14});
		break;
	default:
		usio_log.warning("BNGRW-USIO unknown MIFARE command: 0x%02x", data[1]);
		bngrw_send_ack();
		break;
	}
}

void usb_device_usio::bngrw_cmd_commthru()
{
	bngrw_send_response(m_bngrw_request[6], {0x01});
}

void usb_device_usio::bngrw_cmd_select()
{
	bngrw_bridge_send_event("event select");
	bngrw_send_response(m_bngrw_request[6], {0x00});
}

void usb_device_usio::bngrw_cmd_deselect()
{
	bngrw_bridge_send_event("event deselect");
	bngrw_send_response(m_bngrw_request[6], {0x01, 0x00});
}

void usb_device_usio::bngrw_cmd_release()
{
	bngrw_bridge_send_event("event release");
	bngrw_send_response(m_bngrw_request[6], {0x01, 0x00});
}

std::array<u8, 16>& usb_device_usio::bngrw_block(u16 block)
{
	auto [it, inserted] = m_bngrw_card.blocks.try_emplace(block);
	return it->second;
}

void usb_device_usio::bngrw_felica_read(std::span<const u8> data)
{
	// FeliCa Read Without Encryption request body (after 0x06 cmd):
	// [idm×8] [service_num] [service×(2*N)] [block_num] [block_desc...]
	if (data.size() < 1 + 8 + 1 + 1)
	{
		bngrw_bridge_send_event("event felica_read err short");
		bngrw_send_response(m_bngrw_request[6], {0x01});
		return;
	}

	const u8 service_num = data[8];
	const usz service_off = 9;
	const usz block_num_off = service_off + service_num * 2;
	if (data.size() <= block_num_off)
	{
		bngrw_bridge_send_event("event felica_read err truncated");
		bngrw_send_response(m_bngrw_request[6], {0x01});
		return;
	}

	const u8 requested = data[block_num_off];
	const u8 block_count = std::min<u8>(requested, 4);
	const u8* descriptors = data.data() + block_num_off + 1;
	const usz desc_avail = data.size() - (block_num_off + 1);

	std::vector<u8> resp;
	resp.reserve(2 + 13 + block_count * 16);
	resp.push_back(0x00); // PN532 status
	const u8 felica_len = static_cast<u8>(13 + block_count * 16);
	resp.push_back(felica_len);
	resp.push_back(0x07); // FeliCa response cmd
	resp.insert(resp.end(), m_bngrw_card.idm.begin(), m_bngrw_card.idm.end());
	resp.push_back(0x00); // status flag 1 (OK)
	resp.push_back(0x00); // status flag 2
	resp.push_back(block_count);

	usz desc_pos = 0;
	for (u8 i = 0; i < block_count; i++)
	{
		u16 block_id = 0;
		// 2-byte form (high bit of byte0 set) or 3-byte form. Use simple parse:
		if (desc_pos + 2 <= desc_avail)
		{
			const u8 d0 = descriptors[desc_pos];
			if (d0 & 0x80)
			{
				block_id = (static_cast<u16>(d0) << 8) | descriptors[desc_pos + 1];
				desc_pos += 2;
			}
			else if (desc_pos + 3 <= desc_avail)
			{
				block_id = static_cast<u16>(descriptors[desc_pos + 1]) | (static_cast<u16>(descriptors[desc_pos + 2]) << 8);
				desc_pos += 3;
			}
			else
			{
				desc_pos = desc_avail;
			}
		}

		const auto& blk = bngrw_block(block_id);
		resp.insert(resp.end(), blk.begin(), blk.end());

		bngrw_bridge_send_event(fmt::format("event felica_read block=0x%04x data=%s",
			block_id, fmt::buf_to_hexstring(blk.data(), blk.size())));
	}

	bngrw_send_response(m_bngrw_request[6], resp);
}

void usb_device_usio::bngrw_felica_write(std::span<const u8> data)
{
	// FeliCa Write Without Encryption request body (after 0x08 cmd):
	// [idm×8] [service_num] [service×(2*N)] [block_num] [block_desc...] [block_data×16*M]
	if (data.size() < 1 + 8 + 1 + 1)
	{
		bngrw_send_response(m_bngrw_request[6], {0x01});
		return;
	}

	const u8 service_num = data[8];
	const usz block_num_off = 9 + service_num * 2;
	if (data.size() <= block_num_off)
	{
		bngrw_send_response(m_bngrw_request[6], {0x01});
		return;
	}

	const u8 block_count = data[block_num_off];
	const u8* descriptors = data.data() + block_num_off + 1;
	const usz remaining = data.size() - (block_num_off + 1);

	// figure out descriptor bytes consumed (2 or 3 per block); the rest is data
	std::vector<u16> block_ids;
	block_ids.reserve(block_count);
	usz desc_pos = 0;
	for (u8 i = 0; i < block_count; i++)
	{
		if (desc_pos + 2 > remaining)
		{
			break;
		}

		const u8 d0 = descriptors[desc_pos];
		if (d0 & 0x80)
		{
			block_ids.push_back((static_cast<u16>(d0) << 8) | descriptors[desc_pos + 1]);
			desc_pos += 2;
		}
		else
		{
			if (desc_pos + 3 > remaining)
			{
				break;
			}
			block_ids.push_back(static_cast<u16>(descriptors[desc_pos + 1]) | (static_cast<u16>(descriptors[desc_pos + 2]) << 8));
			desc_pos += 3;
		}
	}

	const u8* blob = descriptors + desc_pos;
	const usz blob_avail = remaining - desc_pos;
	for (usz i = 0; i < block_ids.size() && (i + 1) * 16 <= blob_avail; i++)
	{
		auto& blk = bngrw_block(block_ids[i]);
		std::memcpy(blk.data(), blob + i * 16, 16);

		bngrw_bridge_send_event(fmt::format("event felica_write block=0x%04x data=%s",
			block_ids[i], fmt::buf_to_hexstring(blk.data(), blk.size())));
	}

	// Response: [status=00][len=12][cmd=0x09][idm×8][flag1=00][flag2=00]
	std::vector<u8> resp;
	resp.reserve(2 + 12);
	resp.push_back(0x00);
	resp.push_back(0x0c);
	resp.push_back(0x09);
	resp.insert(resp.end(), m_bngrw_card.idm.begin(), m_bngrw_card.idm.end());
	resp.push_back(0x00);
	resp.push_back(0x00);

	bngrw_send_response(m_bngrw_request[6], resp);
}

void usb_device_usio::bngrw_cmd_felica(std::span<const u8> data)
{
	// PN532 0xa0 payload: [timeout_lo, timeout_hi, felica_len, felica_cmd, ...]
	if (data.size() < 4)
	{
		bngrw_bridge_send_event("event felica err short");
		bngrw_send_response(m_bngrw_request[6], {0x01});
		return;
	}

	const u8 felica_cmd = data[3];
	const std::span<const u8> body = data.subspan(4);

	switch (felica_cmd)
	{
	case 0x06:
		bngrw_felica_read(body);
		return;
	case 0x08:
		bngrw_felica_write(body);
		return;
	default:
		bngrw_bridge_send_event(fmt::format("event felica unhandled cmd=0x%02x", felica_cmd));
		bngrw_send_response(m_bngrw_request[6], {0x01});
		return;
	}
}

void usb_device_usio::bngrw_bridge_init()
{
	const bngrw_sock_t s = ::socket(AF_INET, SOCK_STREAM, 0);
	if (s == bngrw_invalid_sock)
	{
		usio_log.warning("BNGRW-USIO bridge socket() failed");
		return;
	}

	int yes = 1;
	::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(bngrw_bridge_port);
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

	if (::bind(s, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
	{
		usio_log.warning("BNGRW-USIO bridge bind() failed on port %u (errno=%d)", bngrw_bridge_port, bngrw_sock_errno());
		bngrw_close_sock(s);
		return;
	}

	if (::listen(s, 1) != 0)
	{
		usio_log.warning("BNGRW-USIO bridge listen() failed");
		bngrw_close_sock(s);
		return;
	}

	bngrw_set_nonblocking(s);
	m_bngrw_listen = static_cast<uptr>(s);
	usio_log.notice("BNGRW-USIO bridge listening on 127.0.0.1:%u", bngrw_bridge_port);
}

void usb_device_usio::bngrw_bridge_close_client()
{
	if (m_bngrw_client != ~uptr{0})
	{
		bngrw_close_sock(static_cast<bngrw_sock_t>(m_bngrw_client));
		m_bngrw_client = ~uptr{0};
		m_bngrw_rx_buffer.clear();
		usio_log.notice("BNGRW-USIO bridge client disconnected");
	}
}

void usb_device_usio::bngrw_bridge_send_event(std::string_view line)
{
	if (m_bngrw_client == ~uptr{0})
	{
		return;
	}

	std::string buffer;
	buffer.reserve(line.size() + 1);
	buffer.append(line);
	buffer.push_back('\n');

	const bngrw_sock_t c = static_cast<bngrw_sock_t>(m_bngrw_client);
#ifdef _WIN32
	const int sent = ::send(c, buffer.data(), static_cast<int>(buffer.size()), 0);
#else
	const ssize_t sent = ::send(c, buffer.data(), buffer.size(), MSG_NOSIGNAL);
#endif
	if (sent < 0 && !bngrw_would_block(bngrw_sock_errno()))
	{
		bngrw_bridge_close_client();
	}
}

void usb_device_usio::bngrw_bridge_send_state()
{
	switch (m_bngrw_card.type)
	{
	case bngrw_card_type::none:
		bngrw_bridge_send_event("state none");
		break;
	case bngrw_card_type::mifare:
		bngrw_bridge_send_event(fmt::format("state mifare %s",
			fmt::buf_to_hexstring(m_bngrw_card.uid.data(), m_bngrw_card.uid.size())));
		break;
	case bngrw_card_type::felica:
		bngrw_bridge_send_event(fmt::format("state felica %s %s %s",
			fmt::buf_to_hexstring(m_bngrw_card.idm.data(), m_bngrw_card.idm.size()),
			fmt::buf_to_hexstring(m_bngrw_card.pmm.data(), m_bngrw_card.pmm.size()),
			fmt::buf_to_hexstring(m_bngrw_card.system_code.data(), m_bngrw_card.system_code.size())));
		break;
	}
}

void usb_device_usio::bngrw_bridge_poll()
{
	if (m_bngrw_listen == ~uptr{0})
	{
		return;
	}

	if (m_bngrw_client == ~uptr{0})
	{
		const bngrw_sock_t accepted = ::accept(static_cast<bngrw_sock_t>(m_bngrw_listen), nullptr, nullptr);
		if (accepted != bngrw_invalid_sock)
		{
			bngrw_set_nonblocking(accepted);
			m_bngrw_client = static_cast<uptr>(accepted);
			usio_log.notice("BNGRW-USIO bridge client connected");
			bngrw_bridge_send_event("event hello");
			bngrw_bridge_send_state();
		}
	}

	if (m_bngrw_client == ~uptr{0})
	{
		return;
	}

	char buf[256];
	while (true)
	{
		const bngrw_sock_t c = static_cast<bngrw_sock_t>(m_bngrw_client);
#ifdef _WIN32
		const int n = ::recv(c, buf, static_cast<int>(sizeof(buf)), 0);
#else
		const ssize_t n = ::recv(c, buf, sizeof(buf), 0);
#endif
		if (n > 0)
		{
			m_bngrw_rx_buffer.append(buf, static_cast<usz>(n));
			continue;
		}

		if (n == 0)
		{
			bngrw_bridge_close_client();
			return;
		}

		if (!bngrw_would_block(bngrw_sock_errno()))
		{
			bngrw_bridge_close_client();
			return;
		}

		break;
	}

	usz newline = std::string::npos;
	while ((newline = m_bngrw_rx_buffer.find('\n')) != std::string::npos)
	{
		const std::string line = bngrw_trim(std::string_view(m_bngrw_rx_buffer).substr(0, newline));
		m_bngrw_rx_buffer.erase(0, newline + 1);

		if (line.empty())
		{
			continue;
		}

		const std::vector<std::string> args = bngrw_split(line);
		if (args.empty())
		{
			continue;
		}

		if (args[0] == "none" || args[0] == "clear" || args[0] == "off" || args[0] == "remove")
		{
			m_bngrw_card = {};
			m_bngrw_pending = {};
			m_bngrw_pending_active = false;
			usio_log.notice("BNGRW-USIO card cleared");
			bngrw_bridge_send_event("ok cleared");
			bngrw_bridge_send_state();
		}
		else if (args[0] == "status" || args[0] == "?")
		{
			bngrw_bridge_send_state();
		}
		else if (args[0] == "begin" && args.size() >= 3)
		{
			m_bngrw_pending = {};
			m_bngrw_pending_active = true;

			if (args[1] == "mifare" && bngrw_parse_hex(args[2], m_bngrw_pending.uid))
			{
				m_bngrw_pending.type = bngrw_card_type::mifare;
				bngrw_bridge_send_event("ok begin mifare");
			}
			else if ((args[1] == "felica" || args[1] == "bana") && bngrw_parse_hex(args[2], m_bngrw_pending.idm))
			{
				m_bngrw_pending.type = bngrw_card_type::felica;
				if (args.size() >= 4)
				{
					bngrw_parse_hex(args[3], m_bngrw_pending.pmm);
				}
				if (args.size() >= 5)
				{
					bngrw_parse_hex(args[4], m_bngrw_pending.system_code);
				}
				bngrw_bridge_send_event("ok begin felica");
			}
			else
			{
				m_bngrw_pending_active = false;
				bngrw_bridge_send_event("err begin bad_args");
			}
		}
		else if (args[0] == "block" && args.size() >= 3)
		{
			if (!m_bngrw_pending_active)
			{
				bngrw_bridge_send_event("err block no_pending");
			}
			else
			{
				std::array<u8, 2> id_bytes{};
				std::array<u8, 16> blk{};
				if (!bngrw_parse_hex(args[1], id_bytes) || !bngrw_parse_hex(args[2], blk))
				{
					bngrw_bridge_send_event("err block bad_hex");
				}
				else
				{
					const u16 block_id = static_cast<u16>((id_bytes[0] << 8) | id_bytes[1]);
					m_bngrw_pending.blocks[block_id] = blk;
					bngrw_bridge_send_event(fmt::format("ok block 0x%04x", block_id));
				}
			}
		}
		else if (args[0] == "present" || args[0] == "commit")
		{
			if (!m_bngrw_pending_active || m_bngrw_pending.type == bngrw_card_type::none)
			{
				bngrw_bridge_send_event("err present no_pending");
			}
			else
			{
				m_bngrw_card = std::move(m_bngrw_pending);
				m_bngrw_pending = {};
				m_bngrw_pending_active = false;
				usio_log.notice("BNGRW-USIO card presented (type=%u, %zu blocks)",
					static_cast<u32>(m_bngrw_card.type), m_bngrw_card.blocks.size());
				bngrw_bridge_send_event("ok present");
				bngrw_bridge_send_state();
			}
		}
		else if (args[0] == "cancel" || args[0] == "abort")
		{
			m_bngrw_pending = {};
			m_bngrw_pending_active = false;
			bngrw_bridge_send_event("ok cancel");
		}
		else
		{
			usio_log.warning("BNGRW-USIO bridge unknown command: %s", line);
			bngrw_bridge_send_event(fmt::format("err unknown %s", line));
		}
	}
}

// BNGRW command handling is ported from aic_pico firmware/src/lib/bana.c
// for local Taiko card-reader development. aic_pico's real NFC calls are
// represented here as no-card/failure responses until a host card source
// is connected to this bridge.
void usb_device_usio::bngrw_handle_frame()
{
	if (m_bngrw_request.size() < 7)
	{
		return;
	}

	const u8 len = m_bngrw_request[3];
	if (len < 2 || m_bngrw_request.size() < static_cast<usz>(len) + 7)
	{
		return;
	}

	const u8 dir = m_bngrw_request[5];
	const u8 cmd = m_bngrw_request[6];
	const u8* data = len > 2 ? &m_bngrw_request[7] : nullptr;
	const usz data_size = len - 2;
	const std::span<const u8> payload(data, data_size);

	usio_log.notice("BNGRW-USIO request dir=0x%02x cmd=0x%02x data=%s", dir, cmd, fmt::buf_to_hexstring(data, data_size));

	if (dir != 0xd4)
	{
		bngrw_send_ack();
		return;
	}

	switch (cmd)
	{
	case 0x18:
	case 0x12:
		bngrw_send_simple_response();
		break;
	case 0x0e:
		bngrw_cmd_gpio(payload);
		break;
	case 0x08:
		bngrw_send_response(cmd, {0x00});
		break;
	case 0x06:
		if (data_size > 1 && data[1] == 0x1c)
		{
			bngrw_send_response(cmd, {0xff, 0x3f, 0x0e, 0xf1, 0xff, 0x3f, 0x0e, 0xf1});
		}
		else
		{
			bngrw_send_response(cmd, {0xdc, 0xf4, 0x3f, 0x11, 0x4d, 0x85, 0x61, 0xf1, 0x26, 0x6a, 0x87});
		}
		break;
	case 0x32:
		bngrw_cmd_rf_field(payload);
		break;
	case 0x0c:
		bngrw_send_response(cmd, {0x00, 0x06, 0x00});
		break;
	case 0x4a:
		bngrw_cmd_poll_card();
		break;
	case 0x40:
		bngrw_cmd_mifare(payload);
		break;
	case 0x42:
		bngrw_cmd_commthru();
		break;
	case 0x44:
		bngrw_cmd_deselect();
		break;
	case 0xa0:
		bngrw_cmd_felica(payload);
		break;
	case 0x52:
		bngrw_cmd_release();
		break;
	case 0x54:
		bngrw_cmd_select();
		break;
	default:
		usio_log.warning("Unhandled BNGRW-USIO command: 0x%02x len=0x%02x data=%s", cmd, len, fmt::buf_to_hexstring(data, data_size));
		bngrw_send_ack();
		break;
	}
}

void usb_device_usio::bngrw_feed_bytes(const u8* data, u32 size)
{
	for (u32 i = 0; i < size; i++)
	{
		const u8 byte = data[i];
		if (m_bngrw_request.empty() && byte == 0x55)
		{
			continue;
		}

		m_bngrw_request.push_back(byte);

		if (m_bngrw_request.size() == 3 && (m_bngrw_request[0] != 0x00 || m_bngrw_request[1] != 0x00 || m_bngrw_request[2] != 0xff))
		{
			m_bngrw_request.erase(m_bngrw_request.begin());
			continue;
		}

		if (m_bngrw_request.size() == 6 && m_bngrw_request[3] == 0x00)
		{
			m_bngrw_request.clear();
			continue;
		}

		if (m_bngrw_request.size() >= 5)
		{
			const u8 len = m_bngrw_request[3];
			if (len == 0)
			{
				continue;
			}

			if (static_cast<u8>(len + m_bngrw_request[4]) != 0x00)
			{
				usio_log.warning("BNGRW-USIO bad length checksum: len=0x%02x lcs=0x%02x", len, m_bngrw_request[4]);
				m_bngrw_request.clear();
				continue;
			}

			if (m_bngrw_request.size() == static_cast<usz>(len) + 7)
			{
				bngrw_handle_frame();
				m_bngrw_request.clear();
			}
		}
	}
}

void usb_device_usio::interrupt_transfer(u32 buf_size, u8* buf, u32 endpoint, UsbTransfer* transfer)
{
	transfer->fake = true;
	transfer->expected_result = HC_CC_NOERR;
	transfer->expected_time = get_system_time();
	transfer->expected_count = buf_size;
	constexpr u8 USIO_COMMAND_WRITE = 0x90;
	constexpr u8 USIO_COMMAND_READ  = 0x10;
	constexpr u8 USIO_COMMAND_INIT  = 0xA0;

	static bool expecting_data = false;
	static std::vector<u8> usio_data;
	static u32 response_seek = 0;
	static u8 usio_channel   = 0;
	static u16 usio_register = 0;
	static u16 usio_length   = 0;

	//transfer->fake            = true;
	//transfer->expected_result = HC_CC_NOERR;
	// The latency varies per operation but it doesn't seem to matter for this device so let's go fast!
	//transfer->expected_time = get_timestamp() + 1'000;

	is_used = true;

	switch (endpoint)
	{
	case 0x01:
	{
		// Write endpoint
		//transfer->expected_count = buf_size;
		if (buf_size == 6 && (buf[0] & 0xF0) == USIO_COMMAND_READ)
		{
			u16 reg = *reinterpret_cast<le_t<u16>*>(&buf[2]);
			if (reg == 0x1080)
			{
				response_seek = 0;
				response.clear();
				usio_read(buf[0] & 0xF, reg, *reinterpret_cast<le_t<u16>*>(&buf[4]));
				// 状態を壊さず、読み取りだけ完了して戻る
				return;
			}
		}
		if (expecting_data)
		{
			usio_data.insert(usio_data.end(), buf, buf + buf_size);
			

			if(usio_data.size() >= usio_length)
			{
				expecting_data = false;
				usio_write(usio_channel, usio_register, usio_data);
			}
			return;
		}

		// Commands
		if (buf_size != 6)
		{
			usio_log.error("Expected a command but buf_size != 6");
			return;
		}

		usio_channel  = buf[0] & 0xF;
		usio_register = *reinterpret_cast<le_t<u16>*>(&buf[2]);
		usio_length   = *reinterpret_cast<le_t<u16>*>(&buf[4]);

		if ((buf[0] & USIO_COMMAND_WRITE) == USIO_COMMAND_WRITE)
		{
			usio_log.trace("UsioWrite(Channel: 0x%02X, Register: 0x%04X, Length: 0x%04X)", usio_channel, usio_register, usio_length);
			if (((~(usio_register >> 8)) & 0xF0) != buf[1])
			{
				usio_log.error("Invalid UsioWrite command");
				return;
			}
			expecting_data = true;
			usio_data.clear();
		}
		else if ((buf[0] & USIO_COMMAND_READ) == USIO_COMMAND_READ)
		{
			usio_log.trace("UsioRead(Channel: 0x%02X, Register: 0x%04X, Length: 0x%04X)", usio_channel, usio_register, usio_length);
			response_seek = 0;
			response.clear();
			usio_read(usio_channel, usio_register, usio_length);
		}
		else if ((buf[0] & USIO_COMMAND_INIT) == USIO_COMMAND_INIT)
		{
			usio_log.trace("UsioInit(Channel: 0x%02X, Register: 0x%04X, Length: 0x%04X)", usio_channel, usio_register, usio_length);
			usio_init(usio_channel, usio_register, usio_length);
		}
		else
		{
			usio_log.error("Received an unexpected command: 0x%02X", buf[0]);
		}
		break;
	}
	case 0x82:
	{
		// Read endpoint
		const u32 size = std::min(buf_size, static_cast<u32>(response.size() - response_seek));
		memcpy(buf, response.data() + response_seek, size);
		response_seek += size;
		transfer->expected_count = size;
		break;
	}
	default:
		usio_log.error("Unhandled endpoint: 0x%x", endpoint);
		break;
	}
}

usb_device_bngrw::usb_device_bngrw(const std::array<u8, 7>& location)
	: usb_device_emulated(location)
{
	device = UsbDescriptorNode(USB_DESCRIPTOR_DEVICE,
		UsbDeviceDescriptor{
			.bcdUSB             = 0x0110,
			.bDeviceClass       = 0xff,
			.bDeviceSubClass    = 0x00,
			.bDeviceProtocol    = 0xff,
			.bMaxPacketSize0    = 0x40,
			.idVendor           = 0x0b9a,
			.idProduct          = 0x0900,
			.bcdDevice          = 0x0900,
			.iManufacturer      = 0x01,
			.iProduct           = 0x02,
			.iSerialNumber      = 0x03,
			.bNumConfigurations = 0x01});

	auto& config0 = device.add_node(UsbDescriptorNode(USB_DESCRIPTOR_CONFIG,
		UsbDeviceConfiguration{
			.wTotalLength        = 39,
			.bNumInterfaces      = 0x01,
			.bConfigurationValue = 0x01,
			.iConfiguration      = 0x00,
			.bmAttributes        = 0xc0,
			.bMaxPower           = 0x32
		}));

	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_INTERFACE,
		UsbDeviceInterface{
			.bInterfaceNumber   = 0x00,
			.bAlternateSetting  = 0x00,
			.bNumEndpoints      = 0x03,
			.bInterfaceClass    = 0x00,
			.bInterfaceSubClass = 0x00,
			.bInterfaceProtocol = 0x00,
			.iInterface         = 0x00}));

	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint{
			.bEndpointAddress = 0x01,
			.bmAttributes     = 0x02,
			.wMaxPacketSize   = 0x0040,
			.bInterval        = 0x00}));

	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint{
			.bEndpointAddress = 0x82,
			.bmAttributes     = 0x02,
			.wMaxPacketSize   = 0x0040,
			.bInterval        = 0x00}));

	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint{
			.bEndpointAddress = 0x83,
			.bmAttributes     = 0x03,
			.wMaxPacketSize   = 0x0008,
			.bInterval        = 16}));

	add_string("Namco");
	add_string("H050 USJ(C) PCB rev00");
	add_string("00000000");
}

void usb_device_bngrw::control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer)
{
	transfer->fake = true;
	usb_device_emulated::control_transfer(bmRequestType, bRequest, wValue, wIndex, wLength, buf_size, buf, transfer);
}

void usb_device_bngrw::send_ack()
{
	static constexpr std::array<u8, 6> ack{0x00, 0x00, 0xff, 0x00, 0xff, 0x00};
	m_response.insert(m_response.end(), ack.begin(), ack.end());
}

void usb_device_bngrw::send_response(u8 cmd, std::span<const u8> payload)
{
	std::vector<u8> frame;
	const u8 len = static_cast<u8>(2 + payload.size());
	frame.reserve(static_cast<usz>(len) + 7);
	frame.insert(frame.end(), {0x00, 0x00, 0xff, len, static_cast<u8>(~len + 1), 0xd5, static_cast<u8>(cmd + 1)});
	frame.insert(frame.end(), payload.begin(), payload.end());

	u8 checksum = 0xff;
	for (u8 i = 0; i < len; i++)
	{
		checksum += frame[5 + i];
	}

	frame.push_back(static_cast<u8>(~checksum));
	frame.push_back(0x00);
	m_response.insert(m_response.end(), frame.begin(), frame.end());
	usio_log.notice("BNGRW response cmd=0x%02x data=%s", cmd, fmt::buf_to_hexstring(payload.data(), payload.size()));
}

void usb_device_bngrw::send_response(u8 cmd, std::initializer_list<u8> payload)
{
	send_response(cmd, std::span<const u8>(payload.begin(), payload.size()));
}

void usb_device_bngrw::handle_frame()
{
	if (m_request.size() < 7)
	{
		return;
	}

	const u8 len = m_request[3];
	if (len < 2 || m_request.size() < static_cast<usz>(len) + 7)
	{
		return;
	}

	const u8 dir = m_request[5];
	const u8 cmd = m_request[6];
	const u8* data = len > 2 ? &m_request[7] : nullptr;
	const usz data_size = len - 2;

	usio_log.notice("BNGRW request dir=0x%02x cmd=0x%02x data=%s", dir, cmd, fmt::buf_to_hexstring(data, data_size));

	if (dir != 0xd4)
	{
		send_ack();
		return;
	}

	switch (cmd)
	{
	case 0x18:
	case 0x12:
	case 0x0e:
		send_response(cmd);
		break;
	case 0x08:
		send_response(cmd, {0x00});
		break;
	case 0x06:
		if (data_size > 1 && data[1] == 0x1c)
		{
			send_response(cmd, {0xff, 0x3f, 0x0e, 0xf1, 0xff, 0x3f, 0x0e, 0xf1});
		}
		else
		{
			send_response(cmd, {0xdc, 0xf4, 0x3f, 0x11, 0x4d, 0x85, 0x61, 0xf1, 0x26, 0x6a, 0x87});
		}
		break;
	case 0x32:
		send_response(cmd);
		break;
	case 0x0c:
		send_response(cmd, {0x00, 0x06, 0x00});
		break;
	case 0x4a:
		// No card present. This matches aic_pico's Bandai Namco reader fallback
		// and is enough for hardware tests to prove the reader is alive.
		send_response(cmd, {0x00, 0x00, 0x00});
		break;
	case 0x40:
		send_response(cmd, {0x14});
		break;
	case 0x42:
		send_response(cmd, {0x01});
		break;
	case 0x44:
	case 0x52:
		send_response(cmd, {0x01, 0x00});
		break;
	case 0x54:
		send_response(cmd, {0x00});
		break;
	case 0xa0:
		send_response(cmd, {0x01});
		break;
	default:
		usio_log.warning("Unhandled BNGRW command: 0x%02x len=0x%02x data=%s", cmd, len, fmt::buf_to_hexstring(data, data_size));
		send_ack();
		break;
	}
}

void usb_device_bngrw::feed_bytes(const u8* data, u32 size)
{
	for (u32 i = 0; i < size; i++)
	{
		const u8 byte = data[i];
		if (m_request.empty() && byte == 0x55)
		{
			continue;
		}

		m_request.push_back(byte);

		if (m_request.size() == 3 && (m_request[0] != 0x00 || m_request[1] != 0x00 || m_request[2] != 0xff))
		{
			m_request.erase(m_request.begin());
			continue;
		}

		if (m_request.size() == 6 && m_request[3] == 0x00)
		{
			m_request.clear();
			continue;
		}

		if (m_request.size() >= 5)
		{
			const u8 len = m_request[3];
			if (static_cast<u8>(len + m_request[4]) != 0x00)
			{
				usio_log.warning("BNGRW bad length checksum: len=0x%02x lcs=0x%02x", len, m_request[4]);
				m_request.clear();
				continue;
			}

			if (m_request.size() == static_cast<usz>(len) + 7)
			{
				handle_frame();
				m_request.clear();
			}
		}
	}
}

void usb_device_bngrw::interrupt_transfer(u32 buf_size, u8* buf, u32 endpoint, UsbTransfer* transfer)
{
	transfer->fake            = true;
	transfer->expected_result = HC_CC_NOERR;
	transfer->expected_time   = get_timestamp() + 1'000;

	switch (endpoint)
	{
	case 0x01:
		transfer->expected_count = buf_size;
		usio_log.notice("BNGRW write: %s", fmt::buf_to_hexstring(buf, buf_size));
		feed_bytes(buf, buf_size);
		break;
	case 0x82:
	{
		const u32 size = std::min<u32>(buf_size, ::size32(m_response));
		for (u32 i = 0; i < size; i++)
		{
			buf[i] = m_response.front();
			m_response.pop_front();
		}
		transfer->expected_count = size;
		if (size != 0)
		{
			usio_log.notice("BNGRW read: %s", fmt::buf_to_hexstring(buf, size));
		}
		break;
	}
	case 0x83:
		transfer->expected_count = 0;
		break;
	default:
		usio_log.error("Unhandled BNGRW endpoint: 0x%x", endpoint);
		transfer->expected_count = 0;
		break;
	}
}
