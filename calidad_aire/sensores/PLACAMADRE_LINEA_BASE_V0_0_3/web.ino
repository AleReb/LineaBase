//////////////////////////////////////////////////////// parte web
// ==== Static HTML in flash ====
const char HEADER_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Linea Base SD Manager</title>
  <style>
    body { font-family: Arial; max-width:800px; margin:auto; padding:10px; }
    table { width:100%; border-collapse:collapse; margin-top:10px; }
    th, td { padding:8px; border-bottom:1px solid #ddd; }
    th { background:#f4f4f4; }
    button, input[type=text] { margin:2px; padding:5px 10px; }
    input[type=file], progress { margin-top:10px; width:100%; }
    #uploadStatus { margin-left:10px; }
    pre { background:#f0f0f0; padding:10px; overflow:auto; }
    .error { color: red; }
    .success { color: green; }
    .danger-btn { background-color: #ff6b6b !important; color: white !important; }
    .danger-btn:hover { background-color: #ff5252 !important; }
    .warning { background-color: #fff3cd; border: 1px solid #ffeaa7; padding: 10px; margin: 10px 0; border-radius: 4px; }
    .info { background-color: #d1ecf1; border: 1px solid #bee5eb; padding: 10px; margin: 10px 0; border-radius: 4px; }
  </style>
</head>
<body>
  <h1>LINEA BASE SD Manager</h1>
)rawliteral";

const char JAVASCRIPT_HTML[] PROGMEM = R"rawliteral(
<script>
function navigate(path) {
  window.location.href = '/?path=' + encodeURIComponent(path);
}

function makeFolder() {
  var folderName = document.getElementById('newFolder').value.trim();
  if (!folderName) {
    alert('Please enter a folder name');
    return;
  }
  var currentPath = document.getElementById('currentPath').value;
  
  fetch('/mkdir?path=' + encodeURIComponent(currentPath) + '&dirname=' + encodeURIComponent(folderName))
    .then(response => response.text())
    .then(data => {
      if (data === 'Created') {
        alert('Folder created successfully');
        location.reload();
      } else {
        alert('Failed to create folder: ' + data);
      }
    })
    .catch(error => {
      console.error('Error:', error);
      alert('Error creating folder');
    });
}

function uploadFile() {
  var fileInput = document.getElementById('fileInput');
  var file = fileInput.files[0];
  if (!file) {
    alert('Please select a file');
    return;
  }
  
  var currentPath = document.getElementById('currentPath').value;
  var formData = new FormData();
  formData.append('file', file);
  
  var xhr = new XMLHttpRequest();
  xhr.open('POST', '/upload?path=' + encodeURIComponent(currentPath), true);
  
  xhr.upload.onprogress = function(e) {
    if (e.lengthComputable) {
      var percentComplete = (e.loaded / e.total) * 100;
      document.getElementById('uploadProgress').value = percentComplete;
      document.getElementById('uploadStatus').textContent = Math.round(percentComplete) + '%';
    }
  };
  
  xhr.onload = function() {
    if (xhr.status === 200) {
      document.getElementById('uploadStatus').textContent = 'Upload complete!';
      document.getElementById('uploadStatus').className = 'success';
      setTimeout(() => location.reload(), 1000);
    } else {
      document.getElementById('uploadStatus').textContent = 'Upload failed!';
      document.getElementById('uploadStatus').className = 'error';
    }
  };
  
  xhr.onerror = function() {
    document.getElementById('uploadStatus').textContent = 'Upload error!';
    document.getElementById('uploadStatus').className = 'error';
  };
  
  xhr.send(formData);
}

function downloadFile(filename) {
  var currentPath = document.getElementById('currentPath').value;
  var url = '/download?path=' + encodeURIComponent(currentPath) + '&file=' + encodeURIComponent(filename);
  window.open(url, '_blank');
}

function viewFile(filename) {
  var currentPath = document.getElementById('currentPath').value;
  var url = '/view?path=' + encodeURIComponent(currentPath) + '&file=' + encodeURIComponent(filename);
  window.open(url, '_blank');
}

function deleteFile(filename) {
  var currentPath = document.getElementById('currentPath').value;
  
  // Check if it's a directory by looking for [DIR] button
  var isDirectory = false;
  var buttons = document.querySelectorAll('button');
  for (var i = 0; i < buttons.length; i++) {
    if (buttons[i].textContent === '[DIR]' && buttons[i].onclick && buttons[i].onclick.toString().includes(filename)) {
      isDirectory = true;
      break;
    }
  }
  
  var confirmMessage;
  if (isDirectory) {
    confirmMessage = 'WARNING: This will delete the folder "' + filename + '" and ALL its contents permanently!\n\nType "DELETE" to confirm:';
    var userInput = prompt(confirmMessage);
    if (userInput !== 'DELETE') {
      alert('Deletion cancelled - you must type "DELETE" exactly to confirm folder deletion');
      return;
    }
  } else {
    if (!confirm('Are you sure you want to delete the file "' + filename + '"?')) {
      return;
    }
  }
  
  fetch('/delete?path=' + encodeURIComponent(currentPath) + '&file=' + encodeURIComponent(filename))
    .then(response => response.text())
    .then(data => {
      if (data.includes('Deleted')) {
        alert((isDirectory ? 'Folder' : 'File') + ' deleted successfully');
        location.reload();
      } else {
        alert('Failed to delete: ' + data);
      }
    })
    .catch(error => {
      console.error('Error:', error);
      alert('Error deleting ' + (isDirectory ? 'folder' : 'file'));
    });
}

function renameFile(filename) {
  var newName = prompt('Enter new name for "' + filename + '":', filename);
  if (!newName || newName === filename) {
    return;
  }
  
  var currentPath = document.getElementById('currentPath').value;
  
  fetch('/rename?path=' + encodeURIComponent(currentPath) + '&file=' + encodeURIComponent(filename) + '&newname=' + encodeURIComponent(newName))
    .then(response => response.text())
    .then(data => {
      if (data === 'Renamed') {
        alert('File renamed successfully');
        location.reload();
      } else {
        alert('Failed to rename file: ' + data);
      }
    })
    .catch(error => {
      console.error('Error:', error);
      alert('Error renaming file');
    });
}
</script>
)rawliteral";

const char FOOTER_HTML[] PROGMEM = R"rawliteral(
</body>
</html>
)rawliteral";

// // Ensure AP stays alive
// typedef unsigned long ul;
// ul lastAPCheck = 0;
// void ensureAP() {
//   if (millis() - lastAPCheck < 10000) return;
//   lastAPCheck = millis();
//   if (WiFi.softAPgetStationNum() < 0) {
//     if (DEBUG) Serial.println("[DEBUG] AP down, restarting...");
//     WiFi.softAP(ssid, password);
//   }
// }

// ===== Handlers =====

// List files and directories
void handleList() {
  String path = "/";
  if (server.hasArg("path")) {
    path = server.arg("path");
    if (!path.startsWith("/")) path = "/" + path;
  }
  if (DEBUG) Serial.println("[DEBUG] handleList, path=" + path);

  // Send HTML in chunks to avoid memory issues
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  // Send header
  server.sendContent(FPSTR(HEADER_HTML));

  // Send dynamic content
  server.sendContent("<input type='hidden' id='currentPath' value='" + path + "'>");
  server.sendContent("<h2>Current folder: " + path + "</h2>");

  // Safety warning
  server.sendContent("<div class='warning'><strong>⚠️ Safety Notice:</strong> Deleting folders will permanently remove ALL contents inside them.</div>");

  // Info about supported file types
  server.sendContent("<div class='info'><strong>📁 Supported Files:</strong> View images (jpg, png, gif, bmp), text files (txt, csv, log, json, xml, html, js, css, md)</div>");

  // Navigation controls
  server.sendContent("<div><button onclick=\"navigate('')\">Root</button>");
  if (path != "/") {
    int idx = path.lastIndexOf('/');
    String parent = (idx > 0) ? path.substring(0, idx) : "";
    server.sendContent("<button onclick=\"navigate('" + parent + "')\">Up</button>");
  }
  server.sendContent("<input type='text' id='newFolder' placeholder='New folder name'>");
  server.sendContent("<button onclick='makeFolder()'>Create Folder</button></div>");
  server.sendContent("<div><input type='file' id='fileInput'>");
  server.sendContent("<button onclick='uploadFile()'>Upload</button>");
  server.sendContent("<progress id='uploadProgress' value='0' max='100'></progress>");
  server.sendContent("<span id='uploadStatus'></span></div>");
  server.sendContent("<table><tr><th>Name</th><th>Size</th><th>Actions</th></tr>");

  File dir = SD.open(path);
  if (!dir) {
    if (DEBUG) Serial.println("[ERROR] Failed to open directory: " + path);
    server.sendContent("<tr><td colspan='3'>Failed to open directory</td></tr>");
  } else {
    File file = dir.openNextFile();
    while (file) {
      String name = file.name();
      String size = file.isDirectory() ? "-" : String(file.size());
      if (DEBUG) Serial.println("[DEBUG] Listing: " + name + " (" + size + ")");

      String row = "<tr><td>";
      if (file.isDirectory()) {
        row += "<button onclick=\"navigate('" + path + "/" + name + "')\">[DIR]</button> ";
      }
      row += name + "</td><td>" + size + "</td><td>";
      if (!file.isDirectory()) {
        row += "<button onclick=\"downloadFile('" + name + "')\">Download</button>";

        // Check file extension for view button
        String lowerName = name;
        lowerName.toLowerCase();
        if (lowerName.endsWith(".txt") || lowerName.endsWith(".csv") || lowerName.endsWith(".log") || lowerName.endsWith(".json") || lowerName.endsWith(".xml") || lowerName.endsWith(".html") || lowerName.endsWith(".js") || lowerName.endsWith(".css") || lowerName.endsWith(".md") || lowerName.endsWith(".jpg") || lowerName.endsWith(".jpeg") || lowerName.endsWith(".png") || lowerName.endsWith(".gif") || lowerName.endsWith(".bmp")) {
          row += "<button onclick=\"viewFile('" + name + "')\">View</button>";
        }
        row += "<button onclick=\"deleteFile('" + name + "')\">Delete</button>";
      } else {
        row += "<button onclick=\"deleteFile('" + name + "')\" class=\"danger-btn\">Delete Folder</button>";
      }
      row += "<button onclick=\"renameFile('" + name + "')\">Rename</button>";
      row += "</td></tr>";

      server.sendContent(row);
      file.close();
      file = dir.openNextFile();
    }
    dir.close();
  }

  server.sendContent("</table>");
  server.sendContent(FPSTR(JAVASCRIPT_HTML));
  server.sendContent(FPSTR(FOOTER_HTML));
}

// Download file
void handleDownload() {
  if (DEBUG) Serial.println("[DEBUG] handleDownload");
  if (!server.hasArg("path") || !server.hasArg("file")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  String full = (path == "/") ? "/" + server.arg("file") : path + "/" + server.arg("file");
  if (DEBUG) Serial.println("[DEBUG] download " + full);

  File f = SD.open(full);
  if (!f) {
    Serial.println("[ERROR] file not found:" + full);
    server.send(404, "text/plain", "File not found");
    return;
  }

  server.sendHeader("Content-Disposition", "attachment; filename=\"" + server.arg("file") + "\"");
  server.sendHeader("Content-Length", String(f.size()));
  server.streamFile(f, "application/octet-stream");
  f.close();
}

// View text/csv/image files
void handleView() {
  if (DEBUG) Serial.println("[DEBUG] handleView");
  if (!server.hasArg("path") || !server.hasArg("file")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  String name = server.arg("file");
  String full = (path == "/") ? "/" + name : path + "/" + name;

  File f = SD.open(full);
  if (!f) {
    Serial.println("[ERROR] view not found:" + full);
    server.send(404, "text/plain", "File not found");
    return;
  }

  // Determine content type based on file extension
  String contentType = "text/plain";
  String lowerName = name;
  lowerName.toLowerCase();

  if (lowerName.endsWith(".jpg") || lowerName.endsWith(".jpeg")) {
    contentType = "image/jpeg";
  } else if (lowerName.endsWith(".png")) {
    contentType = "image/png";
  } else if (lowerName.endsWith(".gif")) {
    contentType = "image/gif";
  } else if (lowerName.endsWith(".bmp")) {
    contentType = "image/bmp";
  } else if (lowerName.endsWith(".html") || lowerName.endsWith(".htm")) {
    contentType = "text/html";
  } else if (lowerName.endsWith(".css")) {
    contentType = "text/css";
  } else if (lowerName.endsWith(".js")) {
    contentType = "application/javascript";
  } else if (lowerName.endsWith(".json")) {
    contentType = "application/json";
  } else if (lowerName.endsWith(".xml")) {
    contentType = "text/xml";
  }

  // For images, stream the binary data
  if (contentType.startsWith("image/")) {
    server.sendHeader("Content-Length", String(f.size()));
    server.streamFile(f, contentType);
  } else {
    // For text files, send as text
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, contentType, "");
    char buf[512];
    while (f.available()) {
      size_t n = f.readBytes(buf, sizeof(buf) - 1);
      buf[n] = '\0';
      server.sendContent(buf);
    }
  }
  f.close();
}

// Delete file or directory (with recursive delete for directories)
bool deleteRecursive(String path) {
  if (DEBUG) Serial.println("[DEBUG] deleteRecursive: " + path);

  File file = SD.open(path);
  if (!file) {
    if (DEBUG) Serial.println("[ERROR] Cannot open: " + path);
    return false;
  }

  if (file.isDirectory()) {
    if (DEBUG) Serial.println("[DEBUG] Deleting directory contents: " + path);
    file.rewindDirectory();
    File entry = file.openNextFile();
    while (entry) {
      String entryPath = path + "/" + entry.name();
      if (entry.isDirectory()) {
        // Recursive delete for subdirectories
        if (!deleteRecursive(entryPath)) {
          entry.close();
          file.close();
          return false;
        }
      } else {
        // Delete file
        entry.close();
        if (!SD.remove(entryPath)) {
          if (DEBUG) Serial.println("[ERROR] Failed to delete file: " + entryPath);
          file.close();
          return false;
        }
        if (DEBUG) Serial.println("[DEBUG] Deleted file: " + entryPath);
      }
      entry = file.openNextFile();
    }
    file.close();

    // Remove the empty directory
    if (!SD.rmdir(path)) {
      if (DEBUG) Serial.println("[ERROR] Failed to remove directory: " + path);
      return false;
    }
    if (DEBUG) Serial.println("[DEBUG] Removed directory: " + path);
    return true;
  } else {
    // It's a file
    file.close();
    if (!SD.remove(path)) {
      if (DEBUG) Serial.println("[ERROR] Failed to delete file: " + path);
      return false;
    }
    if (DEBUG) Serial.println("[DEBUG] Deleted file: " + path);
    return true;
  }
}

// Delete file or directory
void handleDelete() {
  if (DEBUG) Serial.println("[DEBUG] handleDelete");
  if (!server.hasArg("path") || !server.hasArg("file")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  String full = (path == "/") ? "/" + server.arg("file") : path + "/" + server.arg("file");

  // Check if it's a directory
  File checkFile = SD.open(full);
  if (!checkFile) {
    server.send(404, "text/plain", "File or directory not found");
    return;
  }

  bool isDir = checkFile.isDirectory();
  checkFile.close();

  bool ok = deleteRecursive(full);

  if (ok) {
    server.send(200, "text/plain", isDir ? "Directory deleted successfully" : "File deleted successfully");
  } else {
    server.send(500, "text/plain", isDir ? "Failed to delete directory" : "Failed to delete file");
  }
}

// Rename file
void handleRename() {
  if (DEBUG) Serial.println("[DEBUG] handleRename");
  if (!server.hasArg("path") || !server.hasArg("file") || !server.hasArg("newname")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  String oldp = (path == "/") ? "/" + server.arg("file") : path + "/" + server.arg("file");
  String newp = (path == "/") ? "/" + server.arg("newname") : path + "/" + server.arg("newname");
  bool ok = SD.rename(oldp, newp);
  server.send(ok ? 200 : 500, "text/plain", ok ? "Renamed" : "Failed to rename");
}

// Create directory
void handleMkdir() {
  if (DEBUG) Serial.println("[DEBUG] handleMkdir");
  if (!server.hasArg("path") || !server.hasArg("dirname")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  String dirp = (path == "/") ? "/" + server.arg("dirname") : path + "/" + server.arg("dirname");
  bool ok = SD.mkdir(dirp);
  server.send(ok ? 200 : 500, "text/plain", ok ? "Created" : "Failed to create directory");
}

// File upload
void handleUpload() {
  if (DEBUG) Serial.println("[DEBUG] handleUpload");
  HTTPUpload &up = server.upload();
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;

  if (up.status == UPLOAD_FILE_START) {
    String fp = (path == "/") ? "/" + up.filename : path + "/" + up.filename;
    if (DEBUG) Serial.println("[DEBUG] Upload start: " + fp);
    uploadFile = SD.open(fp, FILE_WRITE);
    if (!uploadFile) {
      Serial.println("[ERROR] Failed to open file for upload");
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      size_t written = uploadFile.write(up.buf, up.currentSize);
      if (DEBUG) Serial.println("[DEBUG] Written: " + String(written) + " bytes");
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      if (DEBUG) Serial.println("[DEBUG] Upload complete");
    }
  }
}