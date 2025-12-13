import React, { useState } from 'react';
import axios from 'axios';

const AdvancedSearch = () => {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [topK, setTopK] = useState(5);
  const [sortBy, setSortBy] = useState('relevance'); // relevance, confidence, date
  const [filterBy, setFilterBy] = useState('all'); // all, title, section, page
  const [filterValue, setFilterValue] = useState('');

  const handleSearch = async (e) => {
    e.preventDefault();
    if (!query.trim()) return;

    setIsLoading(true);
    try {
      const response = await axios.post('http://localhost:8000/api/v1/search', {
        query: query,
        top_k: parseInt(topK)
      });
      let searchResults = response.data.results;

      // Apply client-side filtering if needed
      if (filterBy !== 'all' && filterValue) {
        searchResults = searchResults.filter(result => {
          const source = result.source;
          switch (filterBy) {
            case 'title':
              return source.title && source.title.toLowerCase().includes(filterValue.toLowerCase());
            case 'section':
              return source.section && source.section.toLowerCase().includes(filterValue.toLowerCase());
            case 'page':
              return source.page && source.page.toString().includes(filterValue);
            default:
              return true;
          }
        });
      }

      // Apply sorting
      searchResults.sort((a, b) => {
        switch (sortBy) {
          case 'confidence':
            return (b.confidence_score || b.score) - (a.confidence_score || a.score);
          case 'date':
            // Assuming we have date information in source
            return 0; // Placeholder - would need actual date info
          case 'relevance':
          default:
            return b.score - a.score;
        }
      });

      setResults(searchResults);
    } catch (error) {
      console.error('Search error:', error);
      setResults([]);
    } finally {
      setIsLoading(false);
    }
  };

  const formatSourceInfo = (source) => {
    return (
      <div className="text-xs text-gray-600 mt-1">
        <span>Source: {source.title || source.source_file || 'Unknown'}</span>
        {source.section && <span>, Section: {source.section}</span>}
        {source.page && <span>, Page: {source.page}</span>}
      </div>
    );
  };

  return (
    <div className="max-w-4xl mx-auto p-6">
      <h2 className="text-2xl font-bold mb-6 text-gray-800">Advanced Search</h2>

      <form onSubmit={handleSearch} className="mb-6 grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
        <div className="lg:col-span-2">
          <input
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="Enter your research query..."
            className="w-full px-4 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
          />
        </div>

        <select
          value={topK}
          onChange={(e) => setTopK(e.target.value)}
          className="px-4 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
        >
          {[3, 5, 10, 15, 20].map(num => (
            <option key={num} value={num}>{num} results</option>
          ))}
        </select>

        <button
          type="submit"
          disabled={isLoading}
          className="px-6 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed"
        >
          {isLoading ? 'Searching...' : 'Search'}
        </button>
      </form>

      <div className="mb-6 flex flex-wrap gap-4">
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">Sort by</label>
          <select
            value={sortBy}
            onChange={(e) => setSortBy(e.target.value)}
            className="px-3 py-1 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value="relevance">Relevance</option>
            <option value="confidence">Confidence</option>
          </select>
        </div>

        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">Filter by</label>
          <select
            value={filterBy}
            onChange={(e) => setFilterBy(e.target.value)}
            className="px-3 py-1 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value="all">All</option>
            <option value="title">Title</option>
            <option value="section">Section</option>
            <option value="page">Page</option>
          </select>
        </div>

        {filterBy !== 'all' && (
          <div className="flex-1 min-w-[200px]">
            <label className="block text-sm font-medium text-gray-700 mb-1">Filter value</label>
            <input
              type="text"
              value={filterValue}
              onChange={(e) => setFilterValue(e.target.value)}
              placeholder={`Enter ${filterBy} to filter by`}
              className="w-full px-3 py-1 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
        )}
      </div>

      {isLoading && (
        <div className="flex justify-center my-8">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
        </div>
      )}

      <div className="space-y-4">
        {results.map((result, index) => (
          <div key={index} className="bg-white p-4 rounded-lg shadow border border-gray-200">
            <div className="flex justify-between items-start">
              <div className="flex-1">
                <p className="text-gray-800">{result.content}</p>
                {formatSourceInfo(result.source)}
              </div>
              <div className="ml-4 text-right">
                <div className="text-sm font-semibold text-blue-600">
                  Score: {(result.score * 100).toFixed(1)}%
                </div>
                {result.confidence_score && (
                  <div className="text-xs text-gray-500">
                    Confidence: {(result.confidence_score * 100).toFixed(1)}%
                  </div>
                )}
              </div>
            </div>
          </div>
        ))}
      </div>

      {results.length === 0 && !isLoading && query && (
        <div className="text-center text-gray-500 mt-8">
          No results found for "{query}". Try different keywords.
        </div>
      )}
    </div>
  );
};

export default AdvancedSearch;