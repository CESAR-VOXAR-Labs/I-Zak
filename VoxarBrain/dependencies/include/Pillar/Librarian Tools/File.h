/**
* The Pillar Library
* Copyright (c) 2013 by Voxar Labs.-UFPE
* Permission to use, copy, modify, distribute and sell this software for any
*     purpose is hereby granted without fee, provided that the above copyright
*     notice appear in all copies and that both that copyright notice and this
*     permission notice appear in supporting documentation.
* The author makes no representations about the
*     suitability of this software for any purpose. It is provided "as is"
*     without express or implied warranty.
*/
#ifndef __File_H__
#define __File_H__

namespace LibTools
{
	class File
	{
	protected:

		/** \cond PRIVATE */
		bool binary, loaded, synced;
		
		char* bytes;
		std::stringstream text;

		std::string path;
		std::string name;
		std::string simpleName;
		std::string directory;
		std::string extension;

		unsigned int size;

		/** \endcond */

	public:

		/**
		* Constructor.
		* @param path The file path.
		*/
		File(const std::string& path) : path(path), synced(false), loaded(false), binary(true), bytes(0)
		{
			unsigned int p = this->path.find_last_of('/');
			if (p != std::string::npos)
			{
				this->name = this->path.substr(p + 1);
				this->directory = this->path.substr(0, p + 1);

				std::vector<std::string> subDirs;
				unsigned int p1 = this->directory.find_first_not_of("/");
				unsigned int p2 = 0;

				while (p1 != std::string::npos)
				{
					p2 = this->directory.find_first_of("/", p1);
					subDirs.push_back(this->directory.substr(p1, p2 - p1));
					p1 = this->directory.find_first_not_of("/", p2);
				}
				
				std::string temp = "";

				for (unsigned int i = 0; i < subDirs.size(); i++)
				{
					_mkdir(temp.append(subDirs[i] + "/").c_str());
				}
			}
			else
			{
				this->name = this->path;
				this->directory = "";
			}

			this->simpleName = this->name.substr(0, this->name.find_last_of('.'));

			p = this->name.find_last_of('.');
			if (p != std::string::npos)
			{
				this->extension = this->name.substr(p + 1);
			}
			else
			{
				this->extension = "";
			}

			std::ifstream input(this->path.c_str());
			if (input.is_open())
			{
				input.seekg(0, std::ios::end);
				this->size = (unsigned int)input.tellg();
			}
			else
			{
				this->size = 0;
			}
		}
		
		/**
		* Default destructor.
		*/
		virtual ~File()
		{
			delete[] this->bytes;
		}

		/**
		* Returns the file path.
		* @return The file path.
		*/
		virtual const std::string& getPath() const
		{
			return this->path;
		}

		/**
		* Returns the file name.
		* @return The file name.
		*/
		virtual const std::string& getName() const
		{
			return this->name;
		}

		/**
		* Returns the file name (without extension).
		* @return The file name.
		*/
		virtual const std::string& getSimpleName() const
		{
			return this->simpleName;
		}

		/**
		* Returns the file directory.
		* @return The file directory.
		*/
		virtual const std::string& getDirectory() const
		{
			return this->directory;
		}

		/**
		* Returns the file extension.
		* @return The file extension.
		*/
		virtual const std::string& getExtension() const
		{
			return this->extension;
		}

		/**
		* Returns the file size.
		* @return The file size.
		*/
		virtual const unsigned int getSize() const
		{
			return this->size;
		}

		/**
		* Returns the file content in text format.
		* @return The file content.
		*/
		virtual std::string getText()
		{
			if (!this->loaded)
			{
				this->read();
			}

			if (this->binary && (this->size > 0))
			{
				std::string textTemp;

				if (this->bytes)
					textTemp = this->bytes;

				this->text.str(textTemp);
				this->text.seekp(textTemp.length());

				delete[] this->bytes;
				this->bytes = 0;
				this->binary = false;
			}

			std::string resp = this->text.str();
			return resp;
		}

		/**
		* Returns the file content in binary format.
		* @return The file content.
		*/
		virtual char* getBytes()
		{
			if (!this->loaded)
			{
				this->read();
			}

			if (!this->binary)
			{
				this->bytes = _strdup(this->text.str().c_str());
				this->text.clear();
				this->binary = true;
			}

			return this->bytes;
		}

		/**
		* Sets the file content (in memory).
		* @param text The new content (in text format). 
		*/
		virtual void setText(const std::string& text)
		{
			this->text.str(text);
			this->text << "\0";
			this->text.seekp(text.length());

			delete[] this->bytes;
			this->bytes = 0;
			this->size = text.length();

			this->binary = false;
			this->loaded = true;
			this->synced = false;
		}

		/**
		* Sets the file content (in memory).
		* @param bytes The new content (in binary format).
		* @param size The content size.
		*/
		virtual void setBytes(char* bytes, int size)
		{
			delete[] this->bytes;
			this->text.clear();
			this->bytes = bytes;
			this->size = size;
			this->loaded = true;
			this->binary = true;
			this->synced = false;
		}

		/**
		* Appends some data to the file content (in memory).
		* @param s The data to be appended.
		*/
		virtual void append(const std::string& s)
		{
			if (!loaded)
			{
				this->read();
			}
			if (binary)
			{
				std::string text(this->bytes);
				this->text.str(text);
				this->text.seekp(text.length());

				delete[] this->bytes;
				this->bytes = 0;
				this->binary = false;
			}
			this->text << s << '\0';
			this->text.seekg(0, std::ios::end);
			this->size = (unsigned int)this->text.tellg();
		}

		/**
		* Appends some data to the file content (in disk).
		* @param s The data to be appended.
		*/
		virtual void appendFast(const std::string& s)
		{
			std::ofstream output;
			if (this->binary)
				output.open(this->path.c_str(), std::ios_base::app | std::ios::binary);
			else
				output.open(this->path.c_str(), std::ios_base::app);

			if (output.is_open())
			{
				output << s;

				delete[] this->bytes;
				this->bytes = 0;
				this->text.clear();
				this->binary = false;
				this->loaded = false;
				this->synced = false;
				this->size += s.size();
				output.close();
			}
		}
		
		/**
		* Reads the file content into memory.
		*/
		virtual void read()
		{
			std::ifstream input(this->path.c_str(), std::ifstream::binary);
			if (input.is_open())
			{
				input.seekg(0, std::ios::end);
				unsigned int length = (unsigned int)input.tellg();
				if (length > 0)
				{
					input.seekg(0, std::ios::beg);
					this->bytes = new char[length + 1];
					this->bytes[length] = '\0';
					input.read(this->bytes, length);
					this->size = length;
				}
				input.close();
				this->synced = true;
				this->loaded = true;
				this->binary = true;
			}
		}

		/**
		* Reads part of the file content into memory.
		* @param bPos The begin of the file content that must be read.
		* @param ePos The end of the file content that must be read.
		*/
		virtual void readPart(unsigned int bPos, unsigned int ePos)
		{
			std::ifstream input(this->path.c_str(), std::ios_base::binary);
			input.seekg(bPos);

			this->synced = true;
			this->binary = true;
			this->loaded = true;
			this->bytes = new char[ePos - bPos + 2];
			input.getline(this->bytes, ePos - bPos + 2);
			input.close();
		}

		/**
		* Writes the file content in memory into the disk.
		*/
		virtual void write()
		{
			std::ofstream output;

			if (this->binary)
			{
				output.open(this->path.c_str(), std::ios_base::binary);
			}
			else
			{
				output.open(this->path.c_str());
			}

			if (output.is_open())
			{
				if (this->loaded)
				{
					if (this->binary)
					{
						output.write(bytes, size);
					}
					else
					{
						std::string s = this->text.str();
						output.write(s.c_str(), s.length());						
					}
					this->synced = true;
				}
				output.close();
			}
		}

		/**
		* Writes some data directly into the disk file content.
		* @param s The content to be written
		*/
		virtual void writeFast(const std::string& s)
		{
			std::ofstream output;
			if (this->binary)
			{
				output.open(this->path.c_str(), std::ios_base::binary);
			}
			else
			{
				output.open(this->path.c_str());
			}

			if (output.is_open())
			{
				output.write(s.c_str(), s.size());
				delete[] this->bytes;
				this->bytes = 0;
				this->text.clear();
				this->binary = false;
				this->loaded = false;
				this->synced = false;
				this->size += s.size();
				output.close();
			}
		}	
	};
}
#endif