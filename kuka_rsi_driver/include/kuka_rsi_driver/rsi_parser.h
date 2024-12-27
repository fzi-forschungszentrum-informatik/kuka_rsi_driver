// Copyright 2024 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!\file kuka_rsi_driver/rsi_parser.h
 * \brief Streaming parser for RSI states
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-23
 */
#ifndef KUKA_RSI_DRIVER_RSI_PARSER_H_INCLUDED
#define KUKA_RSI_DRIVER_RSI_PARSER_H_INCLUDED

#include "rsi.h"
#include "rsi_factory.h"

#include <boost/pool/pool.hpp>
#include <cstdint>
#include <expat.h>
#include <functional>
#include <rclcpp/logger.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace kuka_rsi_driver {

/*! \brief Pool-based memory handler for the RSI xml parser
 *
 * Parsing XML requires memory allocations. This class provides a pool-based allocator that can
 * satisfy these in constant time, making them suitable for real-time contexts. Two memory pools are
 * used to efficently handle different allocation sizes.
 */
class XmlMemoryHandler
{
public:
  /*! \brief Create a new memory handler
   *
   * This allocates all the required memory for both memory pools.
   *
   * \note This also calls \p activate() after initialization
   */
  XmlMemoryHandler();

  /*! \brief Use this memory handler for future \p malloc() invocations
   */
  void activate();

  /*! \brief Allocate memory for a given size
   *
   * Depending on \p size, the smaller or large pool is selected.
   *
   * \throws std::bad_alloc If \p activate() was not called before
   * \throws std::bad_alloc If \p size is larger than the larger memory pool
   */
  static void* malloc(std::size_t size);

  /*! \brief Realloc dummy
   *
   * This function does not perform any reallocation and is only provided for completeness.
   *
   * \throws std::bad_alloc always
   */
  static void* realloc(void* ptr, std::size_t size);

  /*! \brief Free memory allocated by \p malloc()
   *
   * If \p ptr is not \p nullptr, the correct memory pool is identified and \p ptr is freed there.
   *
   * \throws std::bad_alloc If \p activate() was not called before
   * \throws std::bad_alloc If \p ptr does not originate from this memory handler
   */
  static void free(void* ptr);

private:
  static XmlMemoryHandler* m_handler;

  using MemoryPool = boost::pool<boost::default_user_allocator_new_delete>;
  MemoryPool m_pool_small;
  MemoryPool m_pool_large;
};

/*! \brief A simplified stream-based XML parser
 *
 * This only handles documents of the structure
 *
 *     <root_tag>           <!-- One specified root document that is known in advance -->
 *       <abc>foo</abc>     <!-- Elements without attributes -->
 *       <foo abc="bar" />  <!-- Elements without text -->
 *     </root_tag>
 *
 * In order to avoid allocations, the parser provides the buffer that is parsed.
 */
class XmlParser
{
public:
  /*! \brief Create a new parser
   *
   * \param scope_filter Which root tag should be expected. Callbacks will not be called outside of
   * its scope.
   * \param log Logger to use.
   * \param buf_size Size of parsing buffer to allocate. Only input within this size can be parsed.
   */
  explicit XmlParser(const std::string& scope_filter,
                     rclcpp::Logger log,
                     std::size_t buf_size = 1024);
  ~XmlParser();

  XmlParser(const XmlParser&)            = delete;
  XmlParser& operator=(const XmlParser&) = delete;

  /*! \brief Access the buffer that is parsed
   *
   * This buffer should be filled with the document before calling \p parseBuffer().
   *
   * \returns Internal parsing buffer.
   */
  [[nodiscard]] std::span<char> buffer() const;

  /*! \brief Parse the document stored in the buffer
   *
   * \param len Length of document previosly loaded into \p buffer().
   */
  void parseBuffer(std::size_t len);

  //! Callback function for elements with text
  using ElementCallback = std::function<void(std::string_view)>;

  /*! \brief Add a callback for elements without attributes
   *
   * \note \p tag needs to still be usable at the time of parsing
   *
   * \param tag Name of tag to be called for.
   * \param cb Callback to be called during parsing.
   */
  void addElementCb(std::string_view tag, ElementCallback cb);

  //! Callback function for elements with attributes
  using AttributeCallback = std::function<void(const char**)>;

  /*! \brief Add a callback for elements with attributes
   *
   * \note \p tag needs to still be usable at the time of parsing
   *
   * \param tag Name of tag to be called for.
   * \param cb Callback to be called during parsing.
   */
  void addAttributeCb(std::string_view tag, AttributeCallback cb);

private:
  static void XMLCALL characterData(void* user_data, const XML_Char* s, int len);
  static void XMLCALL startElement(void* user_data, const XML_Char* name, const XML_Char** atts);
  static void XMLCALL endElement(void* user_data, const XML_Char* name);

  void setupParser();

  rclcpp::Logger m_log;

  std::string m_scope_filter;

  XmlMemoryHandler m_memory_handler;
  XML_Memory_Handling_Suite m_memory_handling_suite;
  XML_Parser m_parser;
  std::span<char> m_buf;

  std::vector<char> m_character_buf;
  std::size_t m_character_buf_i;

  std::size_t m_scope_lvl;
  bool m_in_scope;

  std::unordered_map<std::string_view, ElementCallback> m_element_cbs;
  std::unordered_map<std::string_view, AttributeCallback> m_attribute_cbs;
};

/*! \brief Stream-based parser for RSI state messages
 */
class RsiParser
{
public:
  /*! \brief Create a new parser
   *
   * \param log Logger to use.
   * \param rsi_factory Factory for pre-allocated RSI states
   * \param buf_size Size of parsing buffer to allocate. Only input within this size can be parsed.
   */
  explicit RsiParser(rclcpp::Logger log, RsiFactory* rsi_factory, std::size_t buf_size = 1024);

  RsiParser(const RsiParser&)            = delete;
  RsiParser& operator=(const RsiParser&) = delete;

  /*! \brief Access the buffer that is parsed
   *
   * This buffer should be filled before calling \p parseBuffer().
   *
   * \returns Internal parsing buffer.
   */
  [[nodiscard]] std::span<char> buffer() const;

  /*! \brief Parse the RSI message stored in the internal parsing buffer
   *
   * \param len Length of document stored in \p buffer().
   *
   * \returns Parsed RSI state.
   */
  [[nodiscard]] std::shared_ptr<RsiState> parseBuffer(std::size_t len);

private:
  enum class ValueType
  {
    POSITION_SETPOINT,
    POSITION_ACTUAL,
    TORQUE
  };

  void onAxisElement(ValueType value_type, const XML_Char** atts);
  void onCartesianElement(ValueType value_type, const XML_Char** atts);
  void onOverwrite(const XML_Char** atts);
  void onProgramState(const XML_Char** atts);
  void onDelay(const XML_Char** atts);
  void onIpoc(std::string_view text);

  rclcpp::Logger m_log;

  XmlParser m_xml_parser;

  RsiFactory* m_rsi_factory;
  std::shared_ptr<RsiState> m_rsi_state;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_RSI_PARSER_H_INCLUDED
